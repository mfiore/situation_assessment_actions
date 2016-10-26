/*
This class checks what actions are currently executable by each human, and publishes this information in
the human_executable_actions topic

*/

#ifndef ACTION_PRECONDITONS_CHECKER
#define ACTION_PRECONDITONS_CHECKER

#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>
#include <utility>

#include <common_msgs/ParameterList.h>
#include <common_msgs/Parameter.h>
#include <action_management_msgs/Action.h>
#include <action_management_msgs/CheckPreconditions.h>
#include <action_management_msgs/SetPostconditions.h>
#include <situation_assessment_actions_msgs/ExecutableActions.h>
#include <situation_assessment_actions_msgs/ExecutableAgentActions.h>

#include <situation_assessment_msgs/QueryDatabase.h>
#include <situation_assessment_msgs/FactList.h>


#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/lexical_cast.hpp>


class ActionPreconditionsChecker {
public:
	ActionPreconditionsChecker(ros::NodeHandle node_handle);
	void monitorLoop();

private:
	void queryObjects(); //contacts the database to get objects possessed by the human

	std::string getHumanObject(std::string human); //returns the object (if any) held by the human
	void setHumanObject(std::string human,std::string object);  //sets that the human holds an object

	bool isConnected(std::string l1,std::string l2); //returns true if 2 locations are connected
	std::map<std::string,std::vector<action_management_msgs::Action> > getMoveActions(); //get movement actions


	ros::NodeHandle node_handle_; 
	ros::ServiceClient database_client_; //used to get human observations
	ros::Subscriber fact_subscriber_; //used if we are looping on a topic to get observations

	ros::Publisher human_executable_actions_pub_; //publisher of human actions

	//a std::map that links an action name with a service to get its preconditions
	//each action present in the system needs to provide a checkPreconditions service
	std::map<std::string,ros::ServiceClient> action_preconditions_services_;

	//values read from parameters
	std::vector<std::string> human_list_; //list of humans of the system
	std::vector<std::string> actions_to_monitor_;   //list of actions that need to be monitored
	std::vector<std::string> object_list_; //list of objects in the system
	std::map<std::string,std::vector<std::string> > object_affordances_; //links an object to possible actions
	std::vector<std::string> locations_; //list of location (where the human can move)
	std::map<std::string,std::vector<std::string> > connected_locations_; //maps that links a location to other connecterd locations


	boost::mutex mutex_human_objects_; //allows synchronism on human objects
	std::map<std::string,std::string> human_objects_;  //std::maps each human to an object that he holds

	std::string robot_name_; //name of the robot

	std::map<std::string,std::string> action_targets_info; //links the target of an action to a parameter type. Ex for the target for action pick could be 'main_object'

};

#endif