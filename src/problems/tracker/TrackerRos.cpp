#define _USE_MATH_DEFINES
#include <math.h>

#include <string>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>  // For finding package path
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
//#include <tf/transform_listener.h>
#include "abt/simRosGetObjectHandle.h"
#include "abt/simRosGetObjectPose.h"

#include "solver/Agent.hpp"

#include "problems/shared/ProgramOptions.hpp"
#include "problems/shared/GridPosition.hpp"
#include "problems/shared/simulate.hpp"

#include "TrackerAction.hpp"
#include "TrackerObservation.hpp"
#include "TrackerOptions.hpp"
#include "TrackerState.hpp"
#include "TrackerTextSerializer.hpp"
#include "TrackerModel.hpp"

using std::cout;
using std::endl;
namespace po = boost::program_options;

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

// ROS variables
ros::Publisher leftWheelsPub;
ros::Publisher rightWheelsPub; 
//tf::StampedTransform robotTf;
ros::ServiceClient vrepHandleClient;
ros::ServiceClient vrepPoseClient;
bool seesTarget;
long robotHandle;

// For grid
double cellSize = 2;  // metres
double gridShift = 10;    // In metres, to avoid negative rows and columns
// number of rows/cols = 2 * gridShift / cellSize

/************** Function prototypes ****************/

// Discretises xy to GridPosition
GridPosition getGridPos(double x, double y);

// Gets xy at middle of GridPosition
Point getPoint(const GridPosition &g);

// Returns current position of robot in metres
Point getCurrPos();

// Returns current yaw of robot in radians
double getCurrYaw();

// Returns current yaw in degrees discretised by 45 degree steps
double getCurrYaw45();

// Send motor commands to move towards goal position (in metres) 
// until goal is reached or timeLimit (seconds) reached
void moveTo(double goalX, double goalY, double timeLimit = 6);

// Send motor commands to turn on the spot until goalYaw (in radians)
// is reached or timeLimit (seconds) reached
void turnTo(double goalYaw, double timeLimit = 6);

// Publish desired wheel speeds on ROS topic
void publishSpeeds(double rightSpeed, double leftSpeed);

// Returns the handle of an object in VREP simulation
// Returns -1 if failure
long getVrepHandle(std::string name);

// Get the pose of an object in VREP simulation
geometry_msgs::PoseStamped getVrepPose(long handle);

// Callback when ROS message from VREP is received
// with information on whether target is visible or not
void visibleCallback(const std_msgs::Int32::ConstPtr& msg);

/********************* Main ************************/

int main(int argc, char **argv)
{

	/**************** ROS init **********************/

	ros::init(argc, argv, "tracker_node");
	ros::NodeHandle node;
	ros::Rate loopRate(10);
    leftWheelsPub = node.advertise<std_msgs::Float64>("left_wheels_speed", 1);
    rightWheelsPub = node.advertise<std_msgs::Float64>("right_wheels_speed", 1);
    ros::Subscriber visibleSub = node.subscribe("/human_visible", 1, visibleCallback);
    //tf::TransformListener tfListener;
    vrepHandleClient = node.serviceClient<abt::simRosGetObjectHandle>("vrep/simRosGetObjectHandle");
    vrepPoseClient = node.serviceClient<abt::simRosGetObjectPose>("vrep/simRosGetObjectPose");

    // Get handle of robot object in VREP
    robotHandle = getVrepHandle("robot");

	/**************** ABT init ***********************/

	tracker::TrackerOptions tracker_options;
	ProgramOptions *options = &tracker_options;

	po::options_description visibleOptions;
    po::options_description allOptions;
    visibleOptions.add(options->getGenericOptions()).add(
            options->getABTOptions()).add(options->getProblemOptions()).add(
            options->getHeuristicOptions());
    allOptions.add(visibleOptions).add(options->getSimulationOptions());

    // Set up positional options using ABT's command line interface
    //po::positional_options_description positional;

    po::variables_map vm;
    //po::store(
     //       po::command_line_parser(argc, argv).options(allOptions).positional(
     //               positional).run(), vm);

    std::string trackerPath = ros::package::getPath("abt") + "/src/problems/tracker/";
    std::string cfgPath = trackerPath + "tests/default.cfg";
    po::store(po::parse_config_file<char>(cfgPath.c_str(), allOptions), vm);
    po::notify(vm);
    load_overrides(vm);

    unsigned long seed = vm["seed"].as<unsigned long>();
    if (seed == 0) {
        seed = std::time(nullptr);
    }
    cout << "Seed: " << seed << endl;
    RandomGenerator randGen;
    randGen.seed(seed);
    randGen.discard(10);
    
    // Prepare environment map and initialise tracker model
    std::vector<std::vector<tracker::TrackerModel::TrackerCellType>> envMap;
    int nRows = 2 * gridShift / cellSize;
    int nCols = nRows;
    envMap.resize(nRows);
    for (int i = 0; i < nRows; i++) {
        envMap[i].resize(nCols);
        for (int j = 0; j < nCols; j++) {
            envMap[i][j] = tracker::TrackerModel::TrackerCellType::EMPTY;
        }
    }
    std::unique_ptr<tracker::TrackerModel> newModel = std::make_unique<tracker::TrackerModel>(&randGen, vm);
    newModel->setEnvMap(envMap);
    tracker::TrackerModel *model = newModel.get();

    // Initialise Solver
    solver::Solver solver(&randGen, std::move(newModel));
    std::unique_ptr<solver::Serializer> serializer(std::make_unique<tracker::TrackerTextSerializer>(&solver));
    solver.setSerializer(std::move(serializer));
    solver.initializeEmpty();

    // Generate policy
    double totT;
    double tStart;
    tStart = abt::clock_ms();
    solver.improvePolicy();
    totT = abt::clock_ms() - tStart;
    cout << "Total solving time: " << totT << "ms" << endl;

    // Initialise Agent
    solver::Agent agent(&solver);

	/******************* Main loop *******************/

	long stepNumber = 0;
	while (ros::ok())
	{

		// Get latest tf data (robot's current position)
		/*
		try {
        	tfListener.lookupTransform("/world", "/robot", ros::Time(0), robotTf);
    	}
    	catch (tf::TransformException ex) {
        	ROS_ERROR("%s", ex.what());
    	}
    	*/

		// Improve the policy
		solver::BeliefNode *currentBelief = agent.getCurrentBelief();
		solver.improvePolicy(currentBelief);

		// Apply action
		std::unique_ptr<solver::Action> action = agent.getPreferredAction();
		
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
                cout << "Invalid action: " << (long)actionType << endl;
                break;
        }
        Point goalPoint = getPoint(newCell);
        moveTo(goalPoint.x, goalPoint.y);
        ros::Duration(0.5).sleep();
        turnTo(newYaw * M_PI/180);

		// Action complete, get resulting observations
		pos = getCurrPos();
		currCell = getGridPos(pos.x, pos.y);
		currYaw = getCurrYaw45();
        tracker::TrackerObservation observation(currCell, currYaw, seesTarget);

        cout << "i: " << observation.getRobotPos().i << 
            " j: " << observation.getRobotPos().j << 
            " yaw: " << observation.getRobotYaw() << 
            " see target: " << observation.seesTarget() << endl;


        // Replenish particles
        solver.replenishChild(agent.getCurrentBelief(), *action, observation);

        // Update agent's current belief
		agent.updateBelief(*action, observation);
		
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

// Gets xy at middle of GridPosition
Point getPoint(const GridPosition &g) {
    double x = (g.j * cellSize) - gridShift + 0.5 * cellSize;
    double y = (-g.i * cellSize) + gridShift - 0.5 * cellSize;
    return Point (x, y);
}

// Returns current position of robot in metres
Point getCurrPos() {
    //float x = robotTf.getOrigin().x();
    //float y = robotTf.getOrigin().y();
    geometry_msgs::PoseStamped robotPose = getVrepPose(robotHandle);
    float x = robotPose.pose.position.x;
    float y = robotPose.pose.position.y;
    return Point(x, y);
}

// Returns current yaw of robot in radians
double getCurrYaw() {
	/*
    tf::Quaternion q = robotTf.getRotation();
    double x = q.x();
    double y = q.y();
    double z = q.z();
    double w = q.w();
    */
    geometry_msgs::PoseStamped robotPose = getVrepPose(robotHandle);
    double x = robotPose.pose.orientation.x;
    double y = robotPose.pose.orientation.y;
    double z = robotPose.pose.orientation.z;
    double w = robotPose.pose.orientation.w;
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
        double thresh = 0.1;
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
        double maxSpeed = 3;

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

// Returns the handle of an object in VREP simulation
// Returns -1 if failure
long getVrepHandle(std::string name) {
	abt::simRosGetObjectHandle handleSrv;
	handleSrv.request.objectName = name;
    vrepHandleClient.call(handleSrv);
    return handleSrv.response.handle;
}

// Get the pose of an object in VREP simulation
geometry_msgs::PoseStamped getVrepPose(long handle) {
	abt::simRosGetObjectPose poseSrv;
	poseSrv.request.handle = handle;
	poseSrv.request.relativeToObjectHandle = -1;
    vrepPoseClient.call(poseSrv);
    return poseSrv.response.pose;
}

// Callback when ROS message from VREP is received
// with information on whether target is visible or not
void visibleCallback(const std_msgs::Int32::ConstPtr& msg) {
    seesTarget = (msg->data == 1); 
}