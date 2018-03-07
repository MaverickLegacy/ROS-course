#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

namespace husky_highlevel_controller {

void scanner_callback()
{

}
HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
	 ros::Subscriber scan_subscriber = nodeHandle.subscribe("scan", 10, scanner_callback);
}

HuskyHighlevelController::~HuskyHighlevelController()
{
}

} /* namespace */
