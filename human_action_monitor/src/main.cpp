#include <ros/ros.h>
#include <human_action_monitor/action_monitors.h>
#include <boost/thread.hpp>

int main(int argc, char** argv) {
	ros::init(argc,argv,"human_action_monitor");
	ros::NodeHandle node_handle_;
	ActionMonitors action_monitors(node_handle_);
	action_monitors.actionLoop();
	return 0;
}