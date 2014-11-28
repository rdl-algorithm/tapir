#ifndef TAG_ROS_HPP_
#define TAG_ROS_HPP_

#include <memory>                   // For unique_ptr
#include <vector>
#include <string>

#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>

#include "problems/shared/ros/TapirNode.hpp"
#include "problems/shared/ros/VrepHelper.hpp"
#include "problems/shared/GridPosition.hpp"

#include "problems/tag/TagAction.hpp"
#include "problems/tag/TagObservation.hpp"
#include "problems/tag/TagOptions.hpp"
#include "problems/tag/TagState.hpp"
#include "problems/tag/TagModel.hpp"

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

class TagNode : public TapirNode<TagOptions, TagModel, TagObservation,
		TagAction> {

public:

	TagNode();
	void initialise();

protected:

	/** A Publisher for publishing the position of the robot. */
	ros::Publisher robotPub_;
	/** A Publisher for publishing the position of the human. */
	ros::Publisher humanPub_;
	/** A Publisher for publishing the full environment map. */
	ros::Publisher envMapPub_;
	/** A Publisher for publishing the robot's belief about the target's position. */
	ros::Publisher beliefPub_;

	/** Subscriber to whether human is visible (same cell) or not */
	ros::Subscriber visibleSub_;
	bool visible_;
	/** Subscriber for obstacle selection in VREP (string message) */
	ros::Subscriber selectSub_;

	/** Listener for retrieving tf data, i.e. robot position */
	std::unique_ptr<tf::TransformListener> tfListener_;
	/** Id of world tf frame */
	std::string worldFrame_;

	/** The width of a single grid cell, in metres. */
	double cellSize_;

	/** A timer for processing loop */
	ros::Timer timer_;

	/** A vector to hold changes that will be passed to the model. */
	std::vector<std::unique_ptr<solver::ModelChange>> changes_;

	/** Wrapper for V-REP services */
	VrepHelper vrepHelper_;

	std::unique_ptr<TagObservation> getObservation() override;
	void applyAction(const TagAction& action) override;

	void timerCallback(const ros::TimerEvent& e);

	/** Get xy from GridPosition */
	Point gridToPoint(const GridPosition&g);

	/** Get GridPosition from xy */
	GridPosition pointToGrid(const Point& p);

	/** Callback when info on which cells in VREP are selected  is received */
	void selectCallback(const std_msgs::String::ConstPtr& msg);

	/** Callback when human visibility info is received */
	void visibleCallback(const std_msgs::Int32::ConstPtr& msg);

	/** Publish robot goal position on ROS topic */
	void publishRobotGoal(double goalX, double goalY);

	/** Publish human goal position on ROS topic */
	void publishHumanGoal(double goalX, double goalY);

	/** Publish environment map as string */
	void publishEnvMap(std::vector<std::vector<TagModel::TagCellType>> const &envMap);

	/** Publish belief about target position on VREP */
	void publishBelief(std::vector<std::vector<float>> proportions);
};

} /* namespace tag */

#endif /* TAG_ROS_HPP_ */
