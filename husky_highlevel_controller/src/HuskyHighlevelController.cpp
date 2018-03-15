#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
namespace husky_highlevel_controller {

void HuskyHighlevelController::registerSubscriber()
{
	//scan_sub = nodeHandle_.subscribe();
	scan_sub = nodeHandle_.subscribe("/scan",10, &HuskyHighlevelController::calc_least_dist,this);
}
HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
	 this->registerSubscriber();
	 this->createPublisher();
}

void HuskyHighlevelController::createPublisher()
{
 vel_publisher = nodeHandle_.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel",10);
}
HuskyHighlevelController::~HuskyHighlevelController()
{
}

void HuskyHighlevelController::calc_least_dist(const sensor_msgs::LaserScan& data)
{
 float min_dist = INFINITY;
 int count =0;
 int len = data.ranges.size();
 for(int i =0 ; i < len; i++)
 {
	 if(data.ranges[i] < min_dist){
		 min_dist = data.ranges[i];
		 count = i;
	 }
 }
 float kp =0.9;
 double alpha = data.angle_min + count * data.angle_increment;

 // Send the velocity command to the robot
 geometry_msgs::Twist velocity_control;
 velocity_control.angular.z = -kp * alpha;
 velocity_control.linear.x = 15;
 vel_publisher.publish(velocity_control);
 // Calculate the final position
 float x = min_dist * cos(alpha);
 float y = -min_dist * sin(alpha);
 ros::Publisher vis_pub = nodeHandle_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
 // create visualization
 visualization_msgs::Marker marker;
 marker.header.frame_id = "base_laser";
 marker.header.stamp = ros::Time();
 marker.ns = "my_namespace";
 marker.id = 0;
 marker.type = visualization_msgs::Marker::SPHERE;
 marker.action = visualization_msgs::Marker::ADD;
 marker.pose.position.x = x;
 marker.pose.position.y = y;
 marker.pose.position.z = 1;
 marker.pose.orientation.x = 0.0;
 marker.pose.orientation.y = 0.0;
 marker.pose.orientation.z = 0.0;
 marker.pose.orientation.w = 1.0;
 marker.scale.x = 1;
 marker.scale.y = 0.1;
 marker.scale.z = 0.1;
 marker.color.a = 1.0; // Don't forget to set the alpha!
 marker.color.r = 0.0;
 marker.color.g = 1.0;
 marker.color.b = 0.0;
 //only if using a MESH_RESOURCE marker type:
 marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
 vis_pub.publish( marker );
 /*
 ROS_INFO("alpha=%f", alpha);
 ROS_INFO("(x,y)=(%f,%f)", x,y);
 ROS_INFO("min_dist:%f", min_dist);
 ROS_INFO("count=%d", count);
 ROS_INFO("zero_count=%f", (data.angle_max / data.angle_increment));
 */
 /*
 ROS_INFO("min_angle: %f", data.angle_min);
 ROS_INFO("max_angle: %f", data.angle_max);
 ROS_INFO("divs: %f", (data.angle_max - data.angle_min) / data.angle_increment);
 ROS_INFO("len: %f", len);
 */
}

} /* namespace */
