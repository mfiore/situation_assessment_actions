/*
This class is used to monitor human actions. It reads from the human_executable_actions topic to understand
which actions can be executed at the moment and then through geometrical reasoning computes which actions
are executed. 
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
	 	const situation_assessment_actions_msgs::ExecutableActions::ConstPtr& msg); //call back for actions whose preconditions are true


	//utility functions that turn the action parameters in a std::map
	std::map<std::string,std::string> getParameterMap(
		std::vector<common_msgs::Parameter> parameter_message); 

	/* 
	//computes the distance between the target of an action and the monitor part of 
	an agent for that action (e.g hand) */
	double getDistance(string agent, string target, string monitor_part); 

	/*movement actions are computed directly from a database, by measuring if the location of an  agent
	has changed. This is done in this function */
	std::vector<action_management_msgs::Action> getMoveActions(); 


	ros::NodeHandle node_handle_; 
	
	//used to get human observations	
	ros::ServiceClient database_query_client_;


	ros::Subscriber executable_actions_subscriber_; //subscriber to actions that can be executed

	// contains the list of executable actions
	vector<situation_assessment_actions_msgs::ExecutableAgentActions> executable_actions_;  
	


	//publisher of executed actions
	ros::Publisher executed_actions_pub_;


	//a map that links an action name with a service to get its postconditions
	map<string,ros::ServiceClient> action_postconditions_services_;

	vector<string> object_list_; //list of objects
	map<string,vector<string> > object_affordances_; //links an object to possible actions
	vector<string> human_list_; //list of humans
	map<string,string> human_locations_; //list of locations
	vector<std::string> actions_to_monitor_; //actions to be monitored
	double trigger_distance_;  //trigger for deciding that an action is done
	string robot_name_;  //name of the robot

	ros::ServiceClient database_client_;  //used to add the action's postconditions to the database

	/*links an action to a joint of a human. Ex. pick is linked to the human's hand, which will 
	be used to compute the distance and infer when the action has been done*/	
	std::map<std::string,std::string> action_monitor_parts_; 
	/*information about the action's target. Ex. the target of the action (mug) is the main_object of
	the pick action*/
	std::map<std::string,std::string> action_targets_info_;

	//threshold during which the human can't execute a second action. Useful to avoid situation where
	//the human is picking/placing objects in a loop
	double time_threshold_;

	//two variables used to keep a timer for each agent after they execute an action
	std::map<std::string,SupervisionTimer*> agent_timers_;
	std::map<std::string,boost::thread*> timers_threads_; 

};


#endif