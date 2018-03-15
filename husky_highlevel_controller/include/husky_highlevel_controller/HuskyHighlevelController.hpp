#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

namespace husky_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class HuskyHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	HuskyHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~HuskyHighlevelController();

private:
	ros::NodeHandle nodeHandle_;
	ros::Subscriber scan_sub;
	ros::Publisher vel_publisher;
	tf2_ros::Buffer buffer1;
	tf2_ros::TransformListener transform_listner {buffer1};

	void registerSubscriber();
	void createPublisher();
	void calc_least_dist(const sensor_msgs::LaserScan& data);
};

} /* namespace */
