/** @file TagVrep.cpp
 *
 * Implements the V-REP + ROS interface for Tag, including live output and live addition and
 * deletion of obstacles.
 */
#define _USE_MATH_DEFINES
#include <math.h>

#include <string>                   // For string, stof
#include <sstream>
#include <vector>
#include <memory>                   // For unique_ptr

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>            // For finding package path
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>

#include "VrepHelper.hpp"
#include "options/option_parser.hpp"
#include "problems/shared/GridPosition.hpp"
#include "problems/shared/simulate.hpp"

#include "problems/tag/TagAction.hpp"
#include "problems/tag/TagObservation.hpp"
#include "problems/tag/TagOptions.hpp"
#include "problems/tag/TagState.hpp"
#include "problems/tag/TagModel.hpp"

using std::cout;
using std::endl;

using namespace tag;

// ROS variables
/** A Publisher for publishing the position of the robot. */
ros::Publisher robotPub;
/** A Publisher for publishing the position of the human. */
ros::Publisher humanPub;
/** A Publisher for publishing the full environment map. */
ros::Publisher envMapPub;
/** A Publisher for publishing the robot's belief about the target's position. */
ros::Publisher beliefPub;

/** An instance of VrepHelper class to facilitate basic communications with V-REP, including
 * starting and stopping the simulation, and moving the robot and human to their initial positions.
 */
VrepHelper vrepHelper;

/** A vector to hold changes that will be passed to the model. */
std::vector<std::unique_ptr<solver::ModelChange>> changes;

/** The width of a single grid cell, in metres. */
double cellSize = 1;

namespace tag {
/** A simple implementation of a 2-D point. */
struct Point {
    /** The x-coordinate of this point. */
    double x;
    /** The y-coordinate of this point. */
    double y;
    /** Constructs a new point at (0, 0). */
    Point() : x(0), y(0) {
    }

    /** Constructs a new point at (argX, argY). */
    Point(double argX, double argY) : x(argX), y(argY) {
    }

    /** Returns the distance from this point to the other point given. */
    double dist(Point const &other) const {
        return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
    }

    /** Returns the squared distance from this point to the other point given. */
    double distSq(Point const &other) const {
        return std::pow(x - other.x, 2) + std::pow(y - other.y, 2);
    }

    /** Returns the angle (in radians) from this point to the other point given. */
    double angle(Point const &other) const {
        return atan2(other.y - y, other.x - x);
    }
};

} /* namespace tag */

/************** Function prototypes ****************/

/** Convert a GridPosition to an (x, y) point. */
Point gridToPoint(const GridPosition& g);

/** Convert an (x, y) point to a GridPosition. */
GridPosition pointToGrid(const Point& p);

/** Publish the desired robot position. */
void publishRobotGoal(double goalX, double goalY);

/** Publish the desired human position. */
void publishHumanGoal(double goalX, double goalY);

/** Publish the full environment map as a string. */
void publishEnvMap(std::vector<std::vector<TagModel::TagCellType>> const &envMap);

/** Publish the agent's belief about the target's position. */
void publishBelief(std::vector<std::vector<float>> proportions);

/** An event listener that responds to cells selected in V-REP, allowing addition and deletion
 * of obstacles via the V-REP interface.
 */
void selectCallback(const std_msgs::String::ConstPtr& msg);

/********************* Main ************************/

/** The main method for the Tag V-REP + ROS interface node.
 *
 * This method creates a solver::Simulator instance to run the basic simulation, and enables
 * communication with V-REP to provide a GUI with real-time output, and the ability to add and
 * remove obstacles while the simulation is running.
 */
int main(int argc, char **argv)
{

	/**************** ROS init **********************/

	ros::init(argc, argv, "tag_node");
	ros::NodeHandle node;
	ros::Rate loopRate(1);	// Hz
    robotPub = node.advertise<geometry_msgs::Point>("robot_goal", 1);
    humanPub = node.advertise<geometry_msgs::Point>("human_goal", 1);
    envMapPub = node.advertise<std_msgs::String>("map", 1);
    beliefPub = node.advertise<std_msgs::String>("target_pos_belief", 1);
    ros::Subscriber selectSub = node.subscribe("selected_cells", 1, selectCallback);

	/**************** TAPIR init ***********************/
    std::string tapirPath = ros::package::getPath("tapir");
    std::string cfgPath = tapirPath + "/problems/tag/default.cfg";
    std::unique_ptr<options::OptionParser> parser = TagOptions::makeParser(true);
	TagOptions tagOptions;
    parser->setOptions(&tagOptions);
    parser->parseCfgFile(cfgPath);
    parser->finalize();
    tagOptions.mapPath = tapirPath + "/problems/tag/" + tagOptions.mapPath;
    tagOptions.vrepScenePath = tapirPath + "/problems/tag/" + tagOptions.vrepScenePath;

    if (tagOptions.seed == 0) {
        tagOptions.seed = std::time(nullptr);
    }
    cout << "Seed: " << tagOptions.seed << endl;
    RandomGenerator randGen;
    randGen.seed(tagOptions.seed);
    randGen.discard(10);

    // Initialise TagModel
    std::unique_ptr<TagModel> newModel = std::make_unique<TagModel>(
        &randGen, std::make_unique<TagOptions>(tagOptions));
    TagModel *model = newModel.get();

    // Initialise Solver
    solver::Solver solver(std::move(newModel));
    solver.initializeEmpty();

    // Generate policy
    double totT;
    double tStart;
    tStart = tapir::clock_ms();
    solver.improvePolicy();
    totT = tapir::clock_ms() - tStart;
    cout << "Total solving time: " << totT << "ms" << endl;

    // Initialise TAPIR's Simulator (not VREP!!)
    std::unique_ptr<TagModel> simulatorModel = std::make_unique<TagModel>(
        &randGen, std::make_unique<TagOptions>(tagOptions));
    solver::Simulator simulator(std::move(simulatorModel), &solver, true);
    //if (hasChanges) {
    //    simulator.loadChangeSequence(changesPath);
    //}
    long nSteps = 5000;
    simulator.setMaxStepCount(nSteps);

    /*************** VREP init *******************/

    vrepHelper.setRosNode(&node);

    // Attempt to load and start correct scene
    bool sceneLoaded = false;
    while (ros::ok()) {
        vrepHelper.stop();
        sceneLoaded = vrepHelper.loadScene(tagOptions.vrepScenePath);
        if (sceneLoaded) {
            cout << "Successfully loaded V-REP scene tag.ttt" << endl;
            break;
        } else {
            cout << "V-REP scene tag.ttt isn't loaded. Retrying..." << endl;
        }
        ros::Duration(1).sleep();
        ros::spinOnce();
    }
    if (!sceneLoaded) {
        ROS_ERROR_STREAM("Failed to load V-REP scene");
        return 1;
    }

    // Move robot and human to starting positions in VREP
    solver::State const &startState =  *(simulator.getCurrentState());
    TagState const &startTagState = static_cast<TagState const &>(startState);
    Point startRobotPos = gridToPoint(startTagState.getRobotPosition());
    Point startHumanPos = gridToPoint(startTagState.getOpponentPosition());
    vrepHelper.moveObject("Bill", startHumanPos.x, startHumanPos.y, 0);
    vrepHelper.moveObject("Bill_goalDummy", startHumanPos.x, startHumanPos.y, 0);
    vrepHelper.moveObject("Robot", startRobotPos.x, startRobotPos.y, 0.25);
    vrepHelper.moveObject("Robot_goalDummy", startRobotPos.x, startRobotPos.y, 0);

    // Start the simulation running so that the initial state looks OK.
    vrepHelper.start();
    // Wait a few seconds for V-REP to catch up with the current status.
    ros::Duration(3).sleep();

	/******************* Main loop *******************/

	long stepNumber = 0;
	while (ros::ok()) {

        // Don't loop if VREP is paused
        if (!vrepHelper.isRunning()) {
            cout << "Waiting for VREP" << endl;
            ros::Duration(1).sleep();
            continue;
        }

        // Process any changes (i.e. cell type changing between WALL and EMPTY)
        // based on user input
        simulator.handleChanges(changes);
        changes.clear();

		// Step simulation
        bool finished = !simulator.stepSimulation();

        // Publish new robot and human positions
        solver::State const &state =  *(simulator.getCurrentState());
      	TagState const &tagState = static_cast<TagState const &>(state);
        Point robotPos = gridToPoint(tagState.getRobotPosition());
        Point humanPos = gridToPoint(tagState.getOpponentPosition());
        publishHumanGoal(humanPos.x, humanPos.y);
        publishRobotGoal(robotPos.x, robotPos.y);

        if (finished) {
            cout << "Finished" << endl;
            break;
        }

        // Publish environment map as string
        publishEnvMap(model->getEnvMap());

        // Publish belief about target position on VREP
        solver::BeliefNode *currentBelief = simulator.getAgent()->getCurrentBelief();
        publishBelief(model->getBeliefProportions(currentBelief));

		stepNumber++;
		ros::spinOnce();
		loopRate.sleep();
	}

	return 0;
}

// Get xy from GridPosition
Point gridToPoint(const GridPosition&g) {
	double x = g.j * cellSize;
	double y = -g.i * cellSize;
	return Point(x, y);
}

// Get GridPosition from xy
GridPosition pointToGrid(const Point& p) {
    int j = p.x / cellSize;
    int i = -p.y / cellSize;
    return GridPosition(i, j);
}

// Publish desired human goal position on ROS topic
void publishRobotGoal(double goalX, double goalY) {
    geometry_msgs::Point msg;
    msg.x = goalX;
    msg.y = goalY;
    msg.z = 0;
	robotPub.publish(msg);
}

// Publish desired human goal position on ROS topic
void publishHumanGoal(double goalX, double goalY) {
    geometry_msgs::Point msg;
    msg.x = goalX;
    msg.y = goalY;
    msg.z = 0;
	humanPub.publish(msg);
}

// Publish environment map as string
void publishEnvMap(std::vector<std::vector<TagModel::TagCellType>> const &envMap) {

    // Publish environment map as string: x, y, WALL (0) or EMPTY (1)
    std::stringstream ss;
    for (std::size_t i = 0; i < envMap.size(); i++) {
        for (std::size_t j = 0; j < envMap[i].size(); j++) {
            Point p = gridToPoint(GridPosition(i, j));
            ss << (float) p.x << ",";
            ss << (float) p.y << ",";
            ss << (float) envMap[i][j] << " ";
        }
    }
    std_msgs::String stringMsg;
    stringMsg.data = ss.str();
    envMapPub.publish(stringMsg);
}

// For receiving info on which cells in VREP are selected
void selectCallback(const std_msgs::String::ConstPtr& msg) {

    // String message format is x,y,selected cell type
    std::stringstream ss(msg->data);
    std::string line;
    changes.clear();
    while (std::getline(ss, line, '\n')) {

        // Extract info from string message
        std::stringstream ss2(line);
        std::string strX, strY, strType;
        std::getline(ss2, strX, ',');
        std::getline(ss2, strY, ',');
        std::getline(ss2, strType, ',');
        float x = std::stof(strX);
        float y = std::stof(strY);

        std::cout << "STRYTPE" << strType << endl;

        // Define TagChange (see TagModel.hpp)
        GridPosition g = pointToGrid(Point(x, y));
        std::unique_ptr<TagChange> change = std::make_unique<TagChange>();
        change->i0 = g.i;
        change->i1 = g.i;
        change->j0 = g.j;
        change->j1 = g.j;
        if (strType == "-1") {
            change->changeType = "Remove Obstacles";
        } else {
            change->changeType = "Add Obstacles";
        }
        changes.push_back(std::move(change));
    }
}

// Publish belief about target position on VREP
void publishBelief(std::vector<std::vector<float>> proportions) {

    // Find maximum value to normalise proportion
    float maxProportion = 0;
    for (std::size_t i = 0; i < proportions.size(); i++) {
        for (std::size_t j = 0; j <proportions[i].size(); j++) {
            if (proportions[i][j] > maxProportion) {
                maxProportion = proportions[i][j];
            }
        }
    }

    // Send message. Format is x,y,normalised proportion
    std::stringstream ss;
    for (std::size_t i = 0; i < proportions.size(); i++) {
        for (std::size_t j = 0; j <proportions[i].size(); j++) {
            if (proportions[i][j] == 0) {
                continue;
            }
            Point p = gridToPoint(GridPosition(i, j));
            ss << (float) p.x << ",";
            ss << (float) p.y << ",";
            ss << proportions[i][j]/maxProportion << " ";
        }
    }
    std_msgs::String stringMsg;
    stringMsg.data = ss.str();
    beliefPub.publish(stringMsg);
}
