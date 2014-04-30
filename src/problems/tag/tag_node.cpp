#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>

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

#include "TagAction.hpp"
#include "TagObservation.hpp"
#include "TagOptions.hpp"
#include "TagState.hpp"
#include "TagTextSerializer.hpp"
#include "TagModel.hpp"

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

// Function prototypes

// Discretises xy to GridPosition
GridPosition getGridPos(double x, double y);

// Returns current position of robot in metres
Point getCurrPos();

// Returns current yaw of robot in radians
double getCurrYaw();

// Send motor commands to move towards goal position (in metres) 
// until goal is reached or timeLimit (seconds) reached
void moveTo(double goalX, double goalY, double timeLimit = 1000);

// Publish desired wheel speeds on ROS topic
void publishSpeeds(double rightSpeed, double leftSpeed);

int main(int argc, char **argv)
{

	// ROS init
	ros::init(argc, argv, "tag_node");
	ros::NodeHandle node;
	ros::Rate loopRate(10);
    leftWheelsPub = node.advertise<std_msgs::Float64>("left_wheels_speed", 1);
    rightWheelsPub = node.advertise<std_msgs::Float64>("right_wheels_speed", 1);
    tf::TransformListener tfl;
    tfListener = &tfl;

	// ABT init (based on solve.hpp)
	tag::TagOptions tag_options;
	ProgramOptions *options = &tag_options;

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

    std::unique_ptr<tag::TagModel> newModel = std::make_unique<tag::TagModel>(&randGen, vm);
    tag::TagModel *model = newModel.get();

    // Initialise Solver
    solver::Solver solver(&randGen, std::move(newModel));
    std::unique_ptr<solver::Serializer> serializer(std::make_unique<tag::TagTextSerializer>(&solver));
    solver.setSerializer(std::move(serializer));
    solver.initializeEmpty();

    // Generate policy
    double totT;
    double tStart;
    tStart = abt::clock_ms();
    solver.genPol(model->getNumberOfHistoriesPerStep(), model->getMaximumDepth());
    totT = abt::clock_ms() - tStart;
    cout << "Total solving time: " << totT << "ms" << endl;

    solver::BeliefNode *currNode = solver.getPolicy()->getRoot();
    std::unique_ptr<solver::State> currState = model->sampleAnInitState();

	// Main loop
	long stepNumber = 0;
	while (ros::ok())
	{
        std::stringstream prevStream;
        if (model->hasVerboseOutput()) {
            cout << endl << endl << "t-" << stepNumber << endl;
            cout << "State: " << *currState << endl;
            cout << "Heuristic Value: " << model->getHeuristicValue(*currState) << endl;
            cout << "Belief #" << currNode->getId() << endl;

            solver::HistoricalData *data = currNode->getHistoricalData();
            if (data != nullptr) {
                cout << endl;
                cout << *data;
                cout << endl;
            }

            model->drawSimulationState(currNode, *currState, cout);

            prevStream << "Before:" << endl;
            solver.printBelief(currNode, prevStream);
        }

		//solver.getStatePool()->createOrGetInfo(*currState);

        // Improve the policy
        solver.improveSolution(currNode, model->getNumberOfHistoriesPerStep(),
        	model->getMaximumDepth());

        if (model->hasVerboseOutput()) {
            std::stringstream newStream;
            newStream << "After:" << endl;
            solver.printBelief(currNode, newStream);
            while (prevStream.good() || newStream.good()) {
                std::string s1, s2;
                std::getline(prevStream, s1);
                std::getline(newStream, s2);
                cout << s1 << std::setw(40 - s1.size()) << "";
                cout << s2 << std::setw(40 - s2.size()) << "";
                cout << endl;
            }
        }

        solver::Model::StepResult result = solver.simAStep(currNode, *currState);
        currState = result.nextState->copy();

        solver::BeliefNode *nextNode = currNode->getChild(*result.action,
                    *result.observation);
        if (nextNode == nullptr) {
            nextNode = solver.addChild(currNode, *result.action,
                    *result.observation, stepNumber);
        }
        currNode = nextNode;
        if (result.isTerminal) {
            cout << "Finished" << endl;
            break;
        }
		
		stepNumber++;
		ros::spinOnce();
		loopRate.sleep();
	}

	return 0;
}

// Discretises xy to GridPosition
/*
GridPosition getGridPos(double x, double y) {
    long row = (-y + gridShift) / cellSize;
    long col = (x + gridShift) / cellSize;
    return GridPosition(row, col);
}*/

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

// Publish desired wheel speeds on ROS topic
void publishSpeeds(double rightSpeed, double leftSpeed) {
    std_msgs::Float64 rightWheelsMsg;
    std_msgs::Float64 leftWheelsMsg;

    rightWheelsMsg.data = rightSpeed;
    leftWheelsMsg.data = leftSpeed;

    rightWheelsPub.publish(rightWheelsMsg);
    leftWheelsPub.publish(leftWheelsMsg);
}