/** @file TagNode.cpp
 *
 * Contains TagNode, which implements the TapirNode interface and acts as a ROS node.
 */
#include "TagNode.hpp"

#include <iostream>
#include <sstream>

namespace tag {

TagNode::TagNode() :
		TapirNode(),
		visible_(false),
		worldFrame_("world"),
		cellSize_(1) {
}

void TagNode::initialise() {
	initialiseBase("tag");

    robotPub_ = node_->advertise<geometry_msgs::Point>("robot_goal", 1);
    humanPub_ = node_->advertise<geometry_msgs::Point>("human_goal", 1);
    envMapPub_ = node_->advertise<std_msgs::String>("map", 1);
    beliefPub_ = node_->advertise<std_msgs::String>("target_pos_belief", 1);

    selectSub_ = node_->subscribe("/selected_cells", 1,
    		&TagNode::selectCallback, this);
    visibleSub_ = node_->subscribe("/human_visible", 1,
    		&TagNode::visibleCallback, this);

    tfListener_ = std::make_unique<tf::TransformListener>();

	ros::Duration timerDuration(2);

	vrepHelper_.setRosNode(node_.get());

	// Attempt to load and start correct scene
	bool sceneLoaded = false;
	while (ros::ok()) {
		vrepHelper_.stop();
		sceneLoaded = vrepHelper_.loadScene("tag", options_.vrepScenePath);
		if (sceneLoaded) {
			std::cout << "Successfully loaded V-REP scene tag.ttt" << std::endl;
			break;
		} else {
			std::cout << "V-REP scene tag.ttt isn't loaded. Retrying..." << std::endl;
		}
		ros::Duration(1).sleep();
		ros::spinOnce();
	}

	// Move robot and human to randomised starting positions in VREP
	std::unique_ptr<solver::State> temp = solverModel_->sampleAnInitState();
	std::unique_ptr<TagState> startState(static_cast<TagState *>(temp.release()));
	Point startRobotPos = gridToPoint(startState->getRobotPosition());
	Point startHumanPos = gridToPoint(startState->getOpponentPosition());
	vrepHelper_.moveObject("Bill", startHumanPos.x, startHumanPos.y, 0);
	vrepHelper_.moveObject("Bill_goalDummy", startHumanPos.x, startHumanPos.y, 0);
	vrepHelper_.moveObject("Robot", startRobotPos.x, startRobotPos.y, 0.25);
	vrepHelper_.moveObject("Robot_goalDummy", startRobotPos.x, startRobotPos.y, 0);

	// Start the simulation running so that the initial state looks OK.
	vrepHelper_.start();
	// Wait a few seconds for V-REP to catch up with the current status.
	ros::Duration(3).sleep();

	lastObservation_ = getObservation();

	timer_ = node_->createTimer(timerDuration, &TagNode::timerCallback, this);
}

void TagNode::timerCallback(const ros::TimerEvent& e) {

	// Don't do anything if VREP is paused
	if (!vrepHelper_.isRunning()) {
		std::cout << "Waiting for VREP" << std::endl;
		return;
	}

	// Process any changes (i.e. cell type changing between WALL and EMPTY)
	// based on user input
	if (changes_.size() > 0) {
		handleChanges(changes_, false);
		changes_.clear();
	}

	// Human movement
	tf::StampedTransform transform;
	tfListener_->lookupTransform(worldFrame_, "Bill", ros::Time(0), transform);
	Point p(transform.getOrigin().x(), transform.getOrigin().y());
	GridPosition g = pointToGrid(p);
	GridPosition g2 = solverModel_->sampleNextOpponentPosition(
			lastObservation_->getPosition(), g);
	Point p2 = gridToPoint(g2);

	// Stop once terminal state reached (human is tagged)
	if (stepNumber_ > 0 && lastAction_->getActionType() == ActionType::TAG &&
			g == lastObservation_->getPosition()) {
		std::cout << "Done." << std::endl;
		ros::shutdown();
		return;
	}

	publishHumanGoal(p2.x, p2.y);

	// Call TAPIR processing function. Defined in TapirNode.hpp (base class)
	processTapir();

	// Publish environment map as string
	publishEnvMap(solverModel_->getEnvMap());

	// Publish belief about target position on ROS
	solver::BeliefNode *currentBelief = agent_->getCurrentBelief();
	publishBelief(solverModel_->getBeliefProportions(currentBelief));
}

std::unique_ptr<TagObservation> TagNode::getObservation() {
	tf::StampedTransform transform;
	tfListener_->lookupTransform(worldFrame_, "Robot", ros::Time(0), transform);
	Point p(transform.getOrigin().x(), transform.getOrigin().y());
	GridPosition g = pointToGrid(p);
	std::unique_ptr<TagObservation> observation =
			std::make_unique<TagObservation>(g, visible_);
	return observation;
}

void TagNode::applyAction(const TagAction& action) {
	const GridPosition& oldPos = lastObservation_->getPosition();
	std::pair<GridPosition, bool> temp = solverModel_->getMovedPos(
			oldPos, action.getActionType());
	Point robotPos = gridToPoint(temp.first);
	publishRobotGoal(robotPos.x, robotPos.y);
}

Point TagNode::gridToPoint(const GridPosition&g) {
	double x = (g.j + 0.5) * cellSize_;
	double y = (-g.i - 0.5) * cellSize_;
	return Point(x, y);
}

GridPosition TagNode::pointToGrid(const Point& p) {
    int j = p.x / cellSize_;
    int i = -p.y / cellSize_;
    return GridPosition(i, j);
}

void TagNode::selectCallback(const std_msgs::String::ConstPtr& msg) {

    // String message format is x,y,selected cell type
    std::stringstream ss(msg->data);
    std::string line;
    changes_.clear();
    while (std::getline(ss, line, '\n')) {

        // Extract info from string message
        std::stringstream ss2(line);
        std::string strX, strY, strType;
        std::getline(ss2, strX, ',');
        std::getline(ss2, strY, ',');
        std::getline(ss2, strType, ',');
        float x = std::stof(strX);
        float y = std::stof(strY);

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
        changes_.push_back(std::move(change));
    }
}

void TagNode::visibleCallback(const std_msgs::Int32::ConstPtr& msg) {
	visible_ = msg->data == 1;
}

void TagNode::publishRobotGoal(double goalX, double goalY) {
    geometry_msgs::Point msg;
    msg.x = goalX;
    msg.y = goalY;
    msg.z = 0;
	robotPub_.publish(msg);
}

void TagNode::publishHumanGoal(double goalX, double goalY) {
    geometry_msgs::Point msg;
    msg.x = goalX;
    msg.y = goalY;
    msg.z = 0;
	humanPub_.publish(msg);
}

void TagNode::publishEnvMap(std::vector<std::vector<TagModel::TagCellType>> const &envMap) {

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
    envMapPub_.publish(stringMsg);
}

void TagNode::publishBelief(std::vector<std::vector<float>> proportions) {

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
    beliefPub_.publish(stringMsg);
}

} /* namespace tag */

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tag_node");
	tag::TagNode tagNode;
	tagNode.initialise();
	ros::spin();
	return 0;
}

