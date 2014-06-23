#define _USE_MATH_DEFINES
#include <math.h>


#include <memory>                       // for unique_ptr
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>  // For finding package path
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
//#include <tf/transform_listener.h>

#include "solver/Agent.hpp"

#include "problems/shared/ProgramOptions.hpp"
#include "problems/shared/GridPosition.hpp"
#include "problems/shared/simulate.hpp"

#include "VrepHelper.hpp"

#include "TrackerAction.hpp"
#include "TrackerObservation.hpp"
#include "TrackerOptions.hpp"
#include "TrackerState.hpp"
#include "TrackerModel.hpp"
#include "TrackerRos.hpp"

using namespace tracker;
using std::cout;
using std::endl;
namespace po = boost::program_options;

namespace tracker {

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

/************** Function prototypes ****************/

// Discretises xy to GridPosition
GridPosition xyToGrid(double x, double y);

// Gets xy at middle of GridPosition
Point gridToPoint(const GridPosition &g);

// Returns current position of robot in metres
Point getCurrPos();

// Returns current yaw of robot in radians
double getCurrYaw();

// Returns current yaw in degrees discretised by 45 degree steps
//int getCurrYaw45();   // In header file

// Send motor commands to move towards goal position (in metres) 
// until goal is reached or timeLimit (seconds) reached
void moveTo(double goalX, double goalY, double timeLimit = 6);

// Send motor commands to turn on the spot until goalYaw (in radians)
// is reached or timeLimit (seconds) reached
void turnTo(double goalYaw, double timeLimit = 6);

// Publish desired wheel speeds on ROS topic
void publishSpeeds(double rightSpeed, double leftSpeed);

// Callback when ROS message from VREP is received
// with information on whether target is visible or not
void visibleCallback(const std_msgs::Int32::ConstPtr& msg);

void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);

} /* namespace tracker */

// ROS variables
ros::Publisher leftWheelsPub;
ros::Publisher rightWheelsPub; 
//tf::StampedTransform robotTf;

bool seesTarget;
long robotHandle;
octomap::OcTree* octomapTree;
geometry_msgs::PoseStamped robotPose;

// For grid
double cellSize = 1;  // metres
double gridShift = 10;    // In metres, to avoid negative rows and columns
// number of rows/cols = 2 * gridShift / cellSize

double sensorHeight = 0;

VrepHelper vrepHelper;

/********************* Main ************************/

int main(int argc, char **argv)
{

	/**************** ROS init **********************/

	ros::init(argc, argv, "tracker_node");
	ros::NodeHandle node;
	ros::Rate loopRate(10);
    leftWheelsPub = node.advertise<std_msgs::Float64>("left_wheels_speed", 1);
    rightWheelsPub = node.advertise<std_msgs::Float64>("right_wheels_speed", 1);
    ros::Publisher beliefPub = node.advertise<std_msgs::String>("target_pos_belief", 1);
    ros::Subscriber visibleSub = node.subscribe("human_visible", 1, visibleCallback);
    ros::Subscriber octomapSub = node.subscribe("octomap_full", 1, octomapCallback);
    //tf::TransformListener tfListener;

    int targetPolicy = 0;
    node.getParam("target_policy", targetPolicy);

    /**************** VREP init **********************/

    vrepHelper.setRosNode(&node);

    // Get handle of objects in VREP
    robotHandle = vrepHelper.getHandle("robot");
    long targetHandle = vrepHelper.getHandle("Bill_goalDummy");

    // Update knowledge of current pose (used by ABT)
    robotPose = vrepHelper.getPose(robotHandle);

	/**************** ABT init ***********************/

	TrackerOptions tracker_options;
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
    std::vector<std::vector<TrackerModel::TrackerCellType>> envMap;
    int nRows = 2 * gridShift / cellSize;
    int nCols = nRows;
    envMap.resize(nRows);
    for (int i = 0; i < nRows; i++) {
        envMap[i].resize(nCols);
        for (int j = 0; j < nCols; j++) {
            envMap[i][j] = TrackerModel::TrackerCellType::EMPTY;
        }
    }
    std::unique_ptr<TrackerModel> newModel = std::make_unique<TrackerModel>(&randGen, vm);
    newModel->setEnvMap(envMap);
    
    // Zones mode
    if (targetPolicy == 1) {
    	std::vector<GridPosition> zones;
    	int startZone = 0;
    	zones.push_back(xyToGrid(7.5, 7.5));
    	zones.push_back(xyToGrid(7.5, -7.5));
    	zones.push_back(xyToGrid(-7.5, -7.5));
    	zones.push_back(xyToGrid(-7.5, 7.5));
    	newModel->setPolicyZones(zones, startZone, 0.3);
    	vrepHelper.moveObject("Bill", 7.5, 7.5, 0);
    	vrepHelper.moveObject("Bill_goalDummy", 7.5, 7.5, 0);
    }
    TrackerModel *model = newModel.get();

    // Initialise Solver
    solver::Solver solver(std::move(newModel));
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

    // Spin once to get octomap
	ros::spinOnce();

	// Get height of sensor
	long sensorHandle = vrepHelper.getHandle("scanner");
	geometry_msgs::PoseStamped sensorPose = vrepHelper.getPose(sensorHandle);
	sensorHeight = sensorPose.pose.position.z;

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

    	// Get current position of robot
        robotPose = vrepHelper.getPose(robotHandle);
    	Point pos = getCurrPos();
    	int currYaw = getCurrYaw45();
        GridPosition currCell = xyToGrid(pos.x, pos.y); 

    	// Get changes to octomap
        if (!octomapTree) {
	    	cout << "Null octomap" << endl;
	    }
    	else {

    		// Octomap server is set to a leaf node resolution of 0.125m (set in octomap server settings)
    		// We want to search at depth that has resolution same size of cell (1m)
    		// Note that in octomap, each child's volume is 1/8 of parent,
    		// i.e. resolution doubles each depth layer
    		int searchDepth = octomapTree->getTreeDepth() - 3;
    		cout << "Octomap search resolution: " << octomapTree->getNodeSize(searchDepth) << "m" << endl;
    		octomapTree->updateInnerOccupancy();	// Update occupancy of inner(branch) nodes
    		solver::StatePool *statePool = solver.getStatePool();
            std::vector<std::unique_ptr<solver::ModelChange>> changes;
    		for (int i = 0; i < envMap.size(); i++) {
    			for (int j = 0; j < envMap[0].size(); j++) {
    				GridPosition cell(i, j);
    				Point point = gridToPoint(cell);

    				octomap::OcTreeNode* node = octomapTree->search(point.x, point.y, 0, searchDepth);
    				bool occupied = false;
    				if (node && currCell != cell) {
	    			   	double occupancy = node->getOccupancy();
	    			   	if (occupancy > 0.7) {
	    			   		occupied = true;
	    			   	}
	    			}

	    			// Add obstacle change
	    			if (occupied && envMap[i][j] != TrackerModel::TrackerCellType::WALL) {
	    				changes.push_back(std::make_unique<TrackerChange>("Add Obstacles", i, i, j, j));
	    				envMap[i][j] = TrackerModel::TrackerCellType::WALL;
	    			}

	    			// Remove obstacle change
	    			if (!occupied && envMap[i][j] != TrackerModel::TrackerCellType::EMPTY) {
	    				changes.push_back(std::make_unique<TrackerChange>("Remove Obstacles", i, i, j, j));
	    				envMap[i][j] = TrackerModel::TrackerCellType::EMPTY;
	    			}
	    		}
	    	}

            // Set the change root appropriately.
            solver.setChangeRoot(agent.getCurrentBelief());

            model->applyChanges(changes, &solver);

            // If the current state is deleted, the simulation is broken!
            /*
            StateInfo const *lastInfo = actualHistory_->getLastEntry()->getStateInfo();
            if (changes::has_flag(lastInfo->changeFlags_, ChangeFlags::DELETED)) {
                debug::show_message("ERROR: Current simulation state deleted!");
                return false;
            }

            // If the changes are not dynamic and a past state is deleted, the simulation is broken.
            if (!areDynamic) {
                for (long i = 0; i < actualHistory_->getLength() - 1; i++) {
                    StateInfo const *info = actualHistory_->getEntry(i)->getStateInfo();
                    if (changes::has_flag(info->changeFlags_, ChangeFlags::DELETED)) {
                        std::ostringstream message;
                        message << "ERROR: Impossible simulation history! Includes ";
                        message << *info->getState();
                        debug::show_message(message.str());
                        return false;
                    }
                }
            }
            */
            // Finally we apply the changes.
		    solver.applyChanges();
	    }

	    // Draw envMap
	    model->drawEnvAndPos(cout, currCell);

		// Improve the policy
		cout << "Improving policy" << endl;
		solver::BeliefNode *currentBelief = agent.getCurrentBelief();
		solver.improvePolicy(currentBelief);

        // Publish belief on target's position. This will be visualised in VREP
        cout << "Publishing belief to ROS" << endl;
        std::vector<std::vector<float>> targetPosBelief = model->getTargetPosBelief(currentBelief);

        // Find maximum value to normalise proportion
        float maxProportion = 0;
        for (std::size_t i = 0; i < targetPosBelief.size(); i++) {
            for (std::size_t j = 0; j <targetPosBelief[i].size(); j++) {
                if (targetPosBelief[i][j] > maxProportion) {
                    maxProportion = targetPosBelief[i][j];
                }
            }
        }
        // Send message. Format is x,y,normalised proportion
        std::stringstream ss;
        for (std::size_t i = 0; i < targetPosBelief.size(); i++) {
            for (std::size_t j = 0; j <targetPosBelief[i].size(); j++) {
                if (targetPosBelief[i][j] == 0)
                    continue;
                Point p = gridToPoint(GridPosition(i, j));
                ss << (float) p.x << ",";
                ss << (float) p.y << ",";
                ss << targetPosBelief[i][j]/maxProportion << " ";
            }
        }
        std_msgs::String stringMsg;
        stringMsg.data = ss.str();
        beliefPub.publish(stringMsg);

		// Apply action
		std::unique_ptr<solver::Action> action = agent.getPreferredAction();		
        TrackerAction const &trackerAction = 
        	static_cast<TrackerAction const &>(*action);
        ActionType actionType = trackerAction.getActionType();

        // Override if collision with wall will occur
        GridPosition newPos = model->getNewPos(currCell, currYaw, actionType);
        if (!model->isValid(newPos)) {
        	ROS_WARN_STREAM("Robot wants to go into wall. Overriding.");
        	actionType = ActionType::WAIT;
        	action = std::make_unique<TrackerAction>(actionType);
        }

        int newYaw = currYaw;
        GridPosition newCell = currCell;
        switch (actionType) {
            case ActionType::FORWARD:
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
            case ActionType::TURN_RIGHT:
            	cout << "TURN_RIGHT" << endl;// DEBUG
                newYaw -= 45;
                break;
            case ActionType::TURN_LEFT:
            	cout << "TURN_LEFT" << endl;// DEBUG
                newYaw += 45;
                break;
            case ActionType::REVERSE:
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
            case ActionType::WAIT:
            	cout << "WAIT" << endl;	// DEBUG
                break;
            default:
                cout << "Invalid action: " << (long)actionType << endl;
                break;
        }
        Point goalPoint = gridToPoint(newCell);
        moveTo(goalPoint.x, goalPoint.y);
        ros::Duration(0.5).sleep();
        turnTo(newYaw * M_PI/180);

        if (!ros::ok())
        	break;

        // For zones mode, target position  is controlled by TrackerModel
		if (targetPolicy == 1) {
			
			geometry_msgs::PoseStamped targetPose = vrepHelper.getPose(targetHandle);
			float x = targetPose.pose.position.x;
    		float y = targetPose.pose.position.y;
			GridPosition targetPos = xyToGrid(x, y);
			TargetState newTargetState = model->getNextTargetState(targetPos, 0);
			Point newTargetP = gridToPoint(newTargetState.pos);
			vrepHelper.moveObject(targetHandle, newTargetP.x, newTargetP.y, 0);

			// Debug: override target visibility
			seesTarget = model->isTargetVisible(newCell, newYaw, targetPos);
		}

		// Action complete, get resulting observations
        robotPose = vrepHelper.getPose(robotHandle);
		pos = getCurrPos();
		currCell = xyToGrid(pos.x, pos.y);
		currYaw = getCurrYaw45();
        TrackerObservation observation(currCell, currYaw, seesTarget);

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


namespace tracker {

// Discretises xy to GridPosition
GridPosition xyToGrid(double x, double y) {
    long row = (-y + gridShift) / cellSize;
    long col = (x + gridShift) / cellSize;
    return GridPosition(row, col);
}

// Gets xy at middle of GridPosition
Point gridToPoint(const GridPosition &g) {
    double x = (g.j * cellSize) - gridShift + 0.5 * cellSize;
    double y = (-g.i * cellSize) + gridShift - 0.5 * cellSize;
    return Point (x, y);
}

// Returns current position of robot in metres
Point getCurrPos() {
    //float x = robotTf.getOrigin().x();
    //float y = robotTf.getOrigin().y();
    //geometry_msgs::PoseStamped robotPose = vrepHelper.getPose(robotHandle);
    float x = robotPose.pose.position.x;
    float y = robotPose.pose.position.y;
    return Point(x, y);
}

GridPosition getCurrCell() {
    Point p = getCurrPos();
    return xyToGrid(p.x, p.y);
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
    double x = robotPose.pose.orientation.x;
    double y = robotPose.pose.orientation.y;
    double z = robotPose.pose.orientation.z;
    double w = robotPose.pose.orientation.w;
    return atan2(2*(x*y + w*z), w*w + x*x - y*y - z*z);
}

// Returns current yaw in degrees discretised by 45 degree steps
int getCurrYaw45() {
	double degrees = getCurrYaw() * 180/M_PI;
	int i = round(degrees/45.0);
	int yaw = i * 45;
	if (yaw == -180)
		return 180;
	return yaw;
}

// Send motor commands to move towards goal position (in metres) 
// until goal is reached or timeLimit (seconds) reached
void moveTo(double goalX, double goalY, double timeLimit) {
    ros::Time startTime = ros::Time::now();
    ros::Rate loopRate(20);
    while (ros::ok()) {

        robotPose = vrepHelper.getPose(robotHandle);

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
    ros::Rate loopRate(20);
    while (ros::ok()) {

        robotPose = vrepHelper.getPose(robotHandle);

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

        double maxSpeed = 0.5 + 1.2 * std::abs(relBearing);

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

// Callback when ROS message from VREP is received
// with information on whether target is visible or not
void visibleCallback(const std_msgs::Int32::ConstPtr& msg) {
    seesTarget = (msg->data == 1); 
}

void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
	octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(*msg);
	octomapTree = dynamic_cast<octomap::OcTree*>(tree);
}

// Returns true if ray cast from start to target is obstructed
bool isRayBlocked(GridPosition start, GridPosition target) {

	if (!octomapTree) {
	    //cout << "Null octomap" << endl;
	    return false;
	}

	// Get coordinates
	Point sp = gridToPoint(start);
	Point tp = gridToPoint(target);
	octomap::point3d origin(sp.x, sp.y, sensorHeight);
	octomap::point3d dir(tp.x - sp.x, tp.y - sp.y, 0);
	octomap::point3d end;

	// Check if octomap node exists here. If not, make one
	// with 50% probability of occupied (unknown occupancy)
	octomap::OcTreeNode* node = octomapTree->search(sp.x, sp.y, 0);
	if (!node) {
		octomapTree->updateNode(origin, false);//octomap::logodds(0.5));
	}

	double maxRange = sp.dist(tp);
	return octomapTree->castRay(origin, dir, end, true, maxRange);
}


} /* namespace tracker */