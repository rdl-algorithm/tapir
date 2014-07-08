#ifndef VREPHELPER_HPP_
#define VREPHELPER_HPP_

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "tapir/VrepInfo.h"

class VrepHelper {

public:

	VrepHelper();
	VrepHelper(ros::NodeHandle *node);
	void setRosNode(ros::NodeHandle *node);

	/**
	 * Attempt to start or unpause VREP simulation 
	 * VREP must already be running. Returns true if success
	 */
	bool start();

	/** Attempt to stop VREP simulation. Returns true if success */
	bool stop();

	/**
	 * Returns the handle of an object in VREP simulation
	 * Returns -1 if failure
	 */
	long getHandle(std::string name);

	/**
	 * Move an object in VREP simulation to a new position
	 * Returns true if success.
	 */
	bool moveObject(std::string name, float x, float y, float z);

	/**
	 * Move an object by handle in VREP simulation to
	 * a new position. Returns true if success.
	 */
	bool moveObject(long handle, float x, float y, float z);

	/** Copy an object in VREP simulation. Returns handle of copied object. */
	long copyObject(long handle);

	/** Get the pose of an object in VREP simulation */
	geometry_msgs::PoseStamped getPose(long handle);

	/** Returns true iff VREP simulation is not stopped */
	bool isRunning();

	/** Loads a V-REP scene (.ttt file). Returns true if success */
	bool loadScene(std::string fileName);

private:

	ros::NodeHandle *node_;
	ros::ServiceClient startClient_;
    ros::ServiceClient stopClient_;
	ros::ServiceClient handleClient_;
	ros::ServiceClient copyClient_;
	ros::ServiceClient moveClient_;
	ros::ServiceClient poseClient_;
	ros::ServiceClient loadClient_;
	ros::Subscriber infoSub_;

	bool running_;

	/** Callback for /vrep/info topic */
	void infoCallback(const tapir::VrepInfo::ConstPtr& msg);

};

#endif /* VREPHELPER_HPP_ */