/*
This class is used to monitor human actions.
*/

#ifndef ACTION_MONITORS_H
#define ACTION_MONITORS_H
#include <ros/ros.h>
#include <string>
#include <vector>
#include <utility>
#include <set>

#include <action_management_msgs/Action.h>
#include <action_management_msgs/ActionList.h>
#include <common_msgs/ParameterList.h>
#include <common_msgs/Parameter.h>
#include <action_management_msgs/CheckPreconditions.h>
#include <action_management_msgs/SetPostconditions.h>

#include <situation_assessment_msgs/QueryDatabase.h>
#include <situation_assessment_actions_msgs/ExecutableActions.h>
#include <situation_assessment_actions_msgs/ExecutableAgentActions.h>


#include <situation_assessment_msgs/FactList.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>

#include "supervision_timer/supervision_timer.h"

using namespace std;

class ActionMonitors {
public:
	ActionMonitors(ros::NodeHandle node_handle);
	void actionLoop();
private:

	void executableActionsCallback(
	 	const situation_assessment_actions_msgs::ExecutableActions::ConstPtr& msg);
	// void ActionMonitors::inferenceCallback(
		// const situation_assessment_actions_msgs::IntentionGraphResult::ConstPtr& msg);
	std::map<std::string,std::string> getParameterMap(
		std::vector<common_msgs::Parameter> parameter_message);
	double getDistance(string agent, string target, string monitor_part);


	std::vector<action_management_msgs::Action> getMoveActions();


	ros::NodeHandle node_handle_; 
	ros::ServiceClient database_query_client_; //used only if we are using the database to get human observation
	ros::Subscriber fact_subscriber_; //used if we are looping on a topic to get observations

	ros::Subscriber executable_actions_subscriber_;
	vector<situation_assessment_actions_msgs::ExecutableAgentActions> executable_actions_;  
	vector<std::string> actions_to_monitor_;

	ros::Publisher executed_actions_pub_;


	//a map that links an action name with a service to get its postconditions
	map<string,ros::ServiceClient> action_postconditions_services_;

	//values read from parameters
	vector<string> object_list_;
	map<string,vector<string> > object_affordances_; //links an object to possible actions
	vector<string> human_list_;
	map<string,vector<string> > human_locations_;


	double trigger_distance_;  //trigger for deciding that an action is done

	string robot_name_; 

	bool use_database_; //parameter that decides if we use the db or a topic to get observations

	ros::ServiceClient database_client_;  //puts facts in the db

	std::map<std::string,std::string> action_monitor_parts_;
	std::map<std::string,std::string> action_targets_;

	double time_threshold_;

	std::map<std::string,SupervisionTimer*> agent_timers_;
	std::map<std::string,boost::thread*> timers_threads_; 

	ros::Subscriber inference_sub_;
};


#endif