#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>

// PCL specific includes
//#include <pcl/ros/conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>

#include <fstream>                      // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, ofstream, endl, ostream, ifstream
#include <iostream>                     // for cout
#include <memory>                       // for unique_ptr
#include <string>                       // for string, char_traits, operator<<
#include <utility>                      // for move                // IWYU pragma: keep
#include <vector>                       // for vector, vector<>::iterator

#include <boost/program_options.hpp>    // for variables_map, options_description, positional_options_description, variable_value, store, basic_command_line_parser, command_line_parser, notify, operator<<, parse_config_file, basic_command_line_parser::basic_command_line_parser<charT>, basic_command_line_parser::options, basic_command_line_parser::positional, basic_command_line_parser::run

#include "global.hpp"                     // for RandomGenerator, make_unique
#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/abstract-problem/State.hpp"             // for operator<<, State
#include "solver/serialization/Serializer.hpp"        // for Serializer
#include "solver/Solver.hpp"
#include "solver/BeliefNode.hpp"
#include "solver/BeliefTree.hpp"

#include "problems/shared/ProgramOptions.hpp"
#include "problems/shared/GridPosition.hpp"

#include "problems/tracker/TrackerModel.hpp"
#include "problems/tracker/TrackerOptions.hpp"
#include "problems/tracker/TrackerTextSerializer.hpp"
#include "problems/tracker/TrackerAction.hpp"
#include "problems/tracker/TrackerObservation.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

using std::cout;
using std::endl;
namespace po = boost::program_options;

ros::Publisher leftWheelsPub;
ros::Publisher rightWheelsPub; 
tf::TransformListener *tfListener;

struct Point {
    double x;
    double y;
    Point() : x(0), y(0) {
    }
    Point(double argX, double argY) : x(argX), y(argY) {
    }

    double dist(Point const &other) const {
        return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
    }

    double distSq(Point const &other) const {
        return std::pow(x - other.x, 2) + std::pow(y - other.y, 2);
    }

    double angle(Point const &other) const {
        return atan2(other.y - y, other.x - x);
    }
};

// For grid
double cellSize = 0.5;  // metres
double gridShift = 20;    // In metres, to avoid negative rows and columns
// number of rows/cols = 2 * gridShift / cellSize

// Function prototypes

// Discretises xy to GridPosition
GridPosition getGridPos(double x, double y);

// Returns current position of robot in metres
Point getCurrPos();

// Returns current yaw of robot in radians
double getCurrYaw();

// Returns current yaw in degrees discretised by 45 degree steps
double getCurrYaw45();

// Send motor commands to move towards goal position (in metres) 
// until goal is reached or timeLimit (seconds) reached
void moveTo(double goalX, double goalY, double timeLimit = 1000);

// Send motor commands to turn on the spot until goalYaw (in radians)
// is reached or timeLimit (seconds) reached
void turnTo(double goalYaw, double timeLimit = 1000);

// Publish desired wheel speeds on ROS topic
void publishSpeeds(double rightSpeed, double leftSpeed);

int main(int argc, char **argv)
{

	// ROS init
	ros::init(argc, argv, "abt_node");
	ros::NodeHandle node;
	ros::Rate loopRate(10);
    leftWheelsPub = node.advertise<std_msgs::Float64>("left_wheels_speed", 1);
    rightWheelsPub = node.advertise<std_msgs::Float64>("right_wheels_speed", 1);
    tf::TransformListener tfl;
    tfListener = &tfl;

	// ABT init (based on solve.hpp)
	tracker::TrackerOptions tracker_options;
	ProgramOptions *options = &tracker_options;

	po::options_description visibleOptions;
    po::options_description allOptions;
    visibleOptions.add(options->getGenericOptions()).add(
            options->getABTOptions()).add(options->getProblemOptions()).add(
            options->getHeuristicOptions());
    allOptions.add(visibleOptions).add(options->getSimulationOptions());

    // Set up positional options
    po::positional_options_description positional;
    positional.add("problem.mapPath", 1);
    positional.add("cfg", 2);
    positional.add("policy", 3);

    po::variables_map vm;
    po::store(
            po::command_line_parser(argc, argv).options(allOptions).positional(
                    positional).run(), vm);
    if (vm.count("help")) {
        cout << "Usage: solve [mapPath] [cfgPath] [policyPath]" << endl;
        cout << visibleOptions << endl;
        return 0;
    }

    std::string cfgPath = vm["cfg"].as<std::string>();
    po::store(po::parse_config_file<char>(cfgPath.c_str(), allOptions), vm);
    po::notify(vm);
    load_overrides(vm);

    std::string polPath = vm["policy"].as<std::string>();
    unsigned long seed = vm["seed"].as<unsigned long>();
    if (seed == 0) {
        seed = std::time(nullptr);
    }
    cout << "Seed: " << seed << endl;
    RandomGenerator randGen;
    randGen.seed(seed);
    randGen.discard(10);

    // Prepare environment map and initialise tracker model
    std::vector<std::vector<tracker::TrackerCellType>> envMap;
    int nRows = 2 * gridShift / cellSize;
    int nCols = nRows;
    envMap.resize(nRows);
    for (int i = 0; i < nRows; i++) {
    	envMap[i].resize(nCols);
    	for (int j = 0; j < nCols; j++) {
    		envMap[i][j] = tracker::TrackerCellType::EMPTY;
    	}
    }
    std::unique_ptr<tracker::TrackerModel> newModel = std::make_unique<tracker::TrackerModel>(&randGen, vm, envMap);
    tracker::TrackerModel *model = newModel.get(); // Note to self: Is this legit?

    // Initialise Solver
    solver::Solver solver(&randGen, std::move(newModel));
    std::unique_ptr<solver::Serializer> serializer(std::make_unique<tracker::TrackerTextSerializer>(&solver));
    solver.setSerializer(std::move(serializer));
    solver.initializeEmpty();

    // Generate policy
    double totT;
    double tStart;
    tStart = abt::clock_ms();
    solver.genPol(model->getNumberOfHistoriesPerStep(), model->getMaximumDepth());
    totT = abt::clock_ms() - tStart;
    cout << "Total solving time: " << totT << "ms" << endl;

    // currNode, i.e. current belief
    solver::BeliefNode *currNode = solver.getPolicy()->getRoot();
    cout << "Num currNode particles: " << currNode->getNumberOfParticles() << endl;//DEBUG

	// Main loop
	long stepNumber = 0;
	while (ros::ok())
	{

		// Get current state then add it to state pool
		//std::unique_ptr<State> currState = getCurrState();
		//solver.getStatePool()->createOrGetInfo(*currentState);

		// Improve the policy
        solver.improveSolution(currNode, model->getNumberOfHistoriesPerStep(),
        	model->getMaximumDepth());

		// Get best action according to solver
		std::unique_ptr<solver::Action> action = currNode->getBestAction();

		// Apply action
        tracker::TrackerAction const &trackerAction =
            static_cast<tracker::TrackerAction const &>(*action);
        tracker::ActionType actionType = trackerAction.getActionType();
        int currYaw = getCurrYaw45();
        int newYaw = currYaw;
        Point pos = getCurrPos();
        GridPosition currCell = getGridPos(pos.x, pos.y); 
        GridPosition newCell = currCell;
        switch (actionType) {
            case tracker::ActionType::FORWARD:
            	cout << "FORWARD" << endl;// DEBUG
                // Precondition: -180 <= currYaw <= 180
                if (std::abs(currYaw) < 90)
                    newCell.j += 1;
                else if (std::abs(currYaw) > 90)
                    newCell.j -= 1;
                if (currYaw > 0 && currYaw < 180)
                    newCell.i -= 1;
                else if (currYaw < 0 && currYaw > -180)
                    newCell.i += 1;
                break;
            case tracker::ActionType::TURN_RIGHT:
            	cout << "TURN_RIGHT" << endl;// DEBUG
                newYaw -= 45;
                break;
            case tracker::ActionType::TURN_LEFT:
            	cout << "TURN_LEFT" << endl;// DEBUG
                newYaw += 45;
                break;
            case tracker::ActionType::REVERSE:
            	cout << "REVERSE" << endl;// DEBUG
                if (std::abs(currYaw) < 90)
                    newCell.j -= 1;
                else if (std::abs(currYaw) > 90)
                    newCell.j += 1;
                if (currYaw > 0 && currYaw < 180)
                    newCell.i += 1;
                else if (currYaw < 0 && currYaw > -180)
                    newCell.i -= 1;
                break;
            case tracker::ActionType::WAIT:
            	cout << "WAIT" << endl;	// DEBUG
                break;
            default:
                std::ostringstream message;
                message << "Invalid action: " << (long)actionType;
                debug::show_message(message.str());
                break;
        }

        double goalX = (newCell.j * cellSize) - gridShift;
        double goalY = (-newCell.i * cellSize) + gridShift;
        moveTo(goalX, goalY);
        turnTo(newYaw * M_PI/180);

		// Get observations
		pos = getCurrPos();
		currCell = getGridPos(pos.x, pos.y);
		currYaw = getCurrYaw45();

        tracker::TrackerObservation observation(currCell, currYaw, false); // TODO

        cout << "BOB1 " << observation.getRobotPos().i << " " << observation.getRobotPos().j << " " << observation.getRobotYaw() << endl;
        

		// Update solver
		solver::BeliefNode *nextNode = currNode->getChild(*action, observation);
		if (nextNode == nullptr) {
			cout <<"NULL"<<endl;
			nextNode = solver.addChild(currNode, *action, observation, stepNumber);
		}
		cout <<"BOB2" <<endl;
		currNode = nextNode;
		cout <<"BOB3" <<endl;
		stepNumber++;
		ros::spinOnce();
		loopRate.sleep();
	}

	return 0;
}

// Discretises xy to GridPosition
GridPosition getGridPos(double x, double y) {
    long row = (-y + gridShift) / cellSize;
    long col = (x + gridShift) / cellSize;
    return GridPosition(row, col);
}

// Returns current position of robot in metres
Point getCurrPos() {

    // Lookup transform of robot relative to world origin
    tf::StampedTransform robotTf;
    try {
        tfListener->lookupTransform("/world", "/robot", ros::Time(0), robotTf);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }

    float x = robotTf.getOrigin().x();
    float y = robotTf.getOrigin().y();
    return Point(x, y);
}

// Returns current yaw of robot in radians
double getCurrYaw() {

    // Lookup transform of robot relative to world origin
    tf::StampedTransform robotTf;
    try {
        tfListener->lookupTransform("/world", "/robot", ros::Time(0), robotTf);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }
    tf::Quaternion q = robotTf.getRotation();
    double x = q.x();
    double y = q.y();
    double z = q.z();
    double w = q.w();
    return atan2(2*(x*y + w*z), w*w + x*x - y*y - z*z);
}

// Returns current yaw in degrees discretised by 45 degree steps
double getCurrYaw45() {
	double degrees = getCurrYaw() * 180/M_PI;
	int i = round(degrees/45.0);
	return i * 45;
}

// Send motor commands to move towards goal position (in metres) 
// until goal is reached or timeLimit (seconds) reached
void moveTo(double goalX, double goalY, double timeLimit) {
    ros::Time startTime = ros::Time::now();
    ros::Rate loopRate(10);
    while (ros::ok()) {

        // Check time limit. Break if passed
        if (ros::Time::now() - startTime > ros::Duration(timeLimit))
            break;

        // Get goal and current position as Points
        Point currPos = getCurrPos();
        Point goalPos(goalX, goalY);

        // Check if goal is reached, break if it is
        double thresh = 0.2;
        if (currPos.dist(goalPos) < thresh)
            break;

        // Find bearing of target point relative to robot's forward
        // Positive clockwise
        double angle = currPos.angle(goalPos);
        double relBearing =  getCurrYaw() - angle;

        // Ensure relative bearing is between -pi and pi
        while (relBearing > 0.5 * M_PI)
            relBearing -= 2 * M_PI;
        while (relBearing < -0.5 * M_PI)
            relBearing += 2 * M_PI;

        // Convert to degrees
        relBearing *= 180/M_PI;

        double rightSpeed, leftSpeed;
        double maxSpeed = 5;

        // Smooth right turn / straight ahead
        if (relBearing >= 0 && relBearing <= 45) {
            rightSpeed = (1 - relBearing/45) * maxSpeed;
            leftSpeed = maxSpeed;
        }

        // Smooth left turn
        else if (relBearing <= 0 && relBearing >= -45) {
            rightSpeed = maxSpeed;
            leftSpeed = (1 + relBearing/45) * maxSpeed;
        }

        // Reverse
        else if (relBearing >= 135 && relBearing <= 180) {
            rightSpeed = (1 + (relBearing - 180)/45) * -maxSpeed;
            leftSpeed = -maxSpeed;
        }
        else if (relBearing <= -135 && relBearing >= -180) {
            rightSpeed = -maxSpeed;
            leftSpeed = (1 - (relBearing + 180)/45) * -maxSpeed;
        }

        // On the spot turn
        else if (relBearing > 45) { // && relBearing <= 135
            rightSpeed = -maxSpeed;
            leftSpeed = maxSpeed;
        }
        else {      // relBearing < -45 && relBearing > -135
            rightSpeed = maxSpeed;
            leftSpeed = -maxSpeed;
        }

        publishSpeeds(rightSpeed, leftSpeed);
        ros::spinOnce();
        loopRate.sleep();
    }

    // Stop moving
    publishSpeeds(0, 0);

}

// Send motor commands to turn on the spot until goalYaw (in radians)
// is reached or timeLimit (seconds) reached
void turnTo(double goalYaw, double timeLimit) {
    ros::Time startTime = ros::Time::now();
    ros::Rate loopRate(10);
    while (ros::ok()) {


        // Check time limit. Break if passed
        if (ros::Time::now() - startTime > ros::Duration(timeLimit))
            break;

        double relBearing = getCurrYaw() - goalYaw;

        // Ensure relative bearing is between -pi and pi
        while (relBearing > 0.5 * M_PI)
            relBearing -= 2 * M_PI;
        while (relBearing < -0.5 * M_PI)
            relBearing += 2 * M_PI;

        // Check if goal is reached, break if it is
        double thresh = 0.2;
        if (std::abs(relBearing) < thresh) 
            break;

        double rightSpeed, leftSpeed;

        double maxSpeed = 0.1 + std::abs(relBearing);

        if (relBearing > 0) { 
            rightSpeed = -maxSpeed;
            leftSpeed = maxSpeed;
        }
        else {
            rightSpeed = maxSpeed;
            leftSpeed = -maxSpeed;
        }

        publishSpeeds(rightSpeed, leftSpeed);
        ros::spinOnce();
        loopRate.sleep();
    }

    // Stop moving
    publishSpeeds(0, 0);
}

// Publish desired wheel speeds on ROS topic
void publishSpeeds(double rightSpeed, double leftSpeed) {
    std_msgs::Float64 rightWheelsMsg;
    std_msgs::Float64 leftWheelsMsg;

    rightWheelsMsg.data = rightSpeed;
    leftWheelsMsg.data = leftSpeed;

    rightWheelsPub.publish(rightWheelsMsg);
    leftWheelsPub.publish(leftWheelsMsg);
}