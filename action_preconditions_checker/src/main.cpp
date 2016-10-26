/*
Entry point for the node.
*/

#include <ros/ros.h>
#include <action_preconditions_checker/action_preconditions_checker.h>
#include <boost/thread.hpp>

int main(int argc, char** argv) {
	ros::init(argc,argv,"action_preconditions_checker");
	ROS_INFO("ACTION_PRECONDITONS_CHECKER - init node");
	ros::NodeHandle node_handle_;
	ActionPreconditionsChecker apc(node_handle_);
	boost::thread t(boost::bind(&ActionPreconditionsChecker::monitorLoop,&apc));
	ros::spin();
	return 0;
}