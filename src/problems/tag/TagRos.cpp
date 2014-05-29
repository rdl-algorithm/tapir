#define _USE_MATH_DEFINES
#include <math.h>

#include <string>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>  // For finding package path
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>

#include "VrepHelper.hpp"

#include "problems/shared/ProgramOptions.hpp"
#include "problems/shared/GridPosition.hpp"
#include "problems/shared/simulate.hpp"

#include "TagAction.hpp"
#include "TagObservation.hpp"
#include "TagOptions.hpp"
#include "TagState.hpp"
#include "TagTextSerializer.hpp"
#include "TagModel.hpp"

using std::cout;
using std::endl;
namespace po = boost::program_options;

// ROS variables
ros::Publisher robotPub;
ros::Publisher humanPub;

VrepHelper vrepHelper;

long obstacleHandle;

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

// Get xy from GridPosition
Point gridToPoint(GridPosition g);

// Publish desired robot goal position on ROS topic
void publishRobotGoal(double goalX, double goalY);

// Publish desired human goal position on ROS topic
void publishHumanGoal(double goalX, double goalY);

// Uses ROS services to create obstacle in VREP simulation
void createVrepObstacle(int row, int col);


/********************* Main ************************/

int main(int argc, char **argv)
{

	/**************** ROS init **********************/

	ros::init(argc, argv, "tag_node");
	ros::NodeHandle node;
	ros::Rate loopRate(1);	// Hz
    robotPub = node.advertise<geometry_msgs::Point>("robot_goal", 1);
    humanPub = node.advertise<geometry_msgs::Point>("human_goal", 1);

	/**************** ABT init ***********************/

	tag::TagOptions tag_options;
	ProgramOptions *options = &tag_options;

	po::options_description visibleOptions;
    po::options_description allOptions;
    visibleOptions.add(options->getGenericOptions()).add(
            options->getABTOptions()).add(options->getProblemOptions()).add(
            options->getHeuristicOptions());
    allOptions.add(visibleOptions).add(options->getSimulationOptions());

    // Set up positional options using ABT's command line interface
    po::positional_options_description positional;
    positional.add("problem.mapPath", 1);
    positional.add("cfg", 2);
    positional.add("policy", 3);

    std::string tagPath = ros::package::getPath("abt") + "/src/problems/tag/";
    std::string argMapPath = tagPath + "tests/maps/map.txt";
    std::string argCfgPath = tagPath + "tests/default.cfg";
    char* poArgv[] = {"solve ", &argMapPath[0], &argCfgPath[0]};

    po::variables_map vm;
    po::store(
            po::command_line_parser(3, poArgv).options(allOptions).positional(
                    positional).run(), vm);

    std::string cfgPath = vm["cfg"].as<std::string>();
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

    // Initialise TagModel
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
    solver.improvePolicy();
    totT = abt::clock_ms() - tStart;
    cout << "Total solving time: " << totT << "ms" << endl;

    // Initialise ABT Simulator (not VREP!!)
    std::unique_ptr<tag::TagModel> simulatorModel = std::make_unique<tag::TagModel>(&randGen, vm);
    solver::Simulator simulator(std::move(simulatorModel), &solver);
    //if (hasChanges) {
    //    simulator.loadChangeSequence(changesPath);
    //}
    long nSteps = 5000;
    simulator.setMaxStepCount(nSteps);

    /*************** VREP init *******************/

    vrepHelper.setRosNode(&node);

    // Generate VREP obstacles through ROS service
    // First stop then start VREP simulation to reset obstacles
    while (!vrepHelper.stop() && ros::ok()) {
    	cout << "Waiting for VREP to be ready..." << endl;
    	loopRate.sleep();
    }

    obstacleHandle = vrepHelper.getHandle("Obstacle");
    while (obstacleHandle == -1 && ros::ok()) {
        cout << "Failed to get handle, is VREP scenario tag.ttt loaded?" << endl;
        obstacleHandle = vrepHelper.getHandle("Obstacle");
        ros::Duration(1).sleep();
    }
    ros::Duration(1.5).sleep();
    vrepHelper.start();
    ros::Duration(1.5).sleep();

    // Move robot and human to starting positions in VREP
    solver::State const &startState =  *(simulator.getCurrentState());
    tag::TagState const &startTagState = static_cast<tag::TagState const &>(startState);
    Point startRobotPos = gridToPoint(startTagState.getRobotPosition());
    Point startHumanPos = gridToPoint(startTagState.getOpponentPosition());
    vrepHelper.moveObject("Bill", startHumanPos.x, startHumanPos.y, 0);
    vrepHelper.moveObject("Bill_goalDummy", startHumanPos.x, startHumanPos.y, 0);
    vrepHelper.moveObject("Robot", startRobotPos.x, startRobotPos.y, 0.25);
    vrepHelper.moveObject("Robot_goalDummy", startRobotPos.x, startRobotPos.y, 0);

    cout <<  "Creating VREP obstacles..." << endl;
    
    // Generate walls from envMap
    std::vector<std::vector<tag::TagModel::TagCellType>> envMap = model->getEnvMap();
    int numRows = envMap.size();
    int numCols = envMap[0].size();
    for (int row = 0; row < numRows; row++) {
        for (int col = 0; col < numCols; col++) {
            if (envMap[row][col] != tag::TagModel::TagCellType::WALL) 
                continue;
            createVrepObstacle(row, col);
        }
    }

    // Generate walls around map
    for (int row = -1; row < numRows + 1; row++) {
        for (int col = -1; col < numCols + 1; col++) {
            if (row == -1 || row == envMap.size() || 
                col == -1 || col == envMap[row].size()) {
                createVrepObstacle(row, col);
            }
        }
    }

	/******************* Main loop *******************/

	long stepNumber = 0;
	while (ros::ok())
	{
        bool finished = !simulator.stepSimulation();
        solver::State const &state =  *(simulator.getCurrentState());
      	tag::TagState const &tagState = static_cast<tag::TagState const &>(state);
        Point robotPos = gridToPoint(tagState.getRobotPosition());
        Point humanPos = gridToPoint(tagState.getOpponentPosition());
        publishRobotGoal(robotPos.x, robotPos.y);
        publishHumanGoal(humanPos.x, humanPos.y);

        if (finished) {
            cout << "Finished" << endl;
            break;
        }
		
		stepNumber++;
		ros::spinOnce();
		loopRate.sleep();
	}

	return 0;
}

// Get xy from GridPosition
Point gridToPoint(GridPosition g) {
	double cellSize = 1;  // metres
	double x = g.j * cellSize;
	double y = -g.i * cellSize;
	return Point(x, y);
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

// Uses ROS services to create obstacle in VREP simulation
void createVrepObstacle(int row, int col) {
    long newHandle = vrepHelper.copyObject(obstacleHandle);
    Point p = gridToPoint(GridPosition(row, col));
    vrepHelper.moveObject(newHandle, p.x, p.y, 0.5);
}
