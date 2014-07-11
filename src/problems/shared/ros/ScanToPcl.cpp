/** @file ScanToPcl.cpp
 *
 * A simple ROS node that subscribes to the laser scan data published by V-REP, and then
 * re-publishes it as a PointCould2 message which can then be used by the Octomap server.
 */

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

/** An event listener to respond to laser scan data from V-REP. */
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

/** This utility instance does the work of converting the laser data to a point cloud. */
laser_geometry::LaserProjection projector;
/** A Publisher for publishing the point cloud. */
ros::Publisher pclPub;

/** The main method for this laser data conversion node, which converts V-REP laser data messages
 *  to PointCloud2 messages.
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "scan_to_pcl_node");
	ros::NodeHandle node;
    pclPub = node.advertise<sensor_msgs::PointCloud2>("cloud_in", 1);
    ros::Subscriber scanSub = node.subscribe("/vrep/front_scan", 1, scanCallback);
    ros::spin();
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    sensor_msgs::PointCloud2 cloud;
    projector.projectLaser(*msg, cloud);
    pclPub.publish(cloud);
}
