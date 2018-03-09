#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include<cmath>
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
}

HuskyHighlevelController::~HuskyHighlevelController()
{
}

void HuskyHighlevelController::calc_least_dist(const sensor_msgs::LaserScan& data)
{
 float min_dist = INFINITY;
 int min_ind =0;
 int len = data.ranges.size();
 for(int i =0 ; i < len; i++)
 {
	 if(data.ranges[i] < min_dist){
		 min_dist = data.ranges[i];
		 min_ind = i;
	 }
 }
 ROS_INFO("\nsmallest_distance: %f", min_dist);

}

} /* namespace */
