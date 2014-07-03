// This node simply subscribes to the laser scan data published
// by VREP, and publishes it as a pointcloud2 message which
// is required by the octomap server

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

laser_geometry::LaserProjection projector;
ros::Publisher pclPub;

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