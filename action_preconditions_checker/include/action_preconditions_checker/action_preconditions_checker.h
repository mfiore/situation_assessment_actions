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

#include <situation_assessment_msgs/QueryDatabase.h>
#include <situation_assessment_msgs/FactList.h>



#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/lexical_cast.hpp>


class ActionPreconditionsChecker {
public:
	ActionPreconditionsChecker(ros::NodeHandle node_handle);
	void start();

private:
	void monitorLoop();
	void databaseLoop();

	void agentFactCallback(const situation_assessment_msgs::FactList::ConstPtr& msg);

	std::string getHumanObject(std::string human); //returns the object (if any) held by the human
	void setHumanObject(std::string human,std::string object);  //sets that the human holds an object


	ros::NodeHandle node_handle_; 
	ros::ServiceClient database_client_; //used only if we are using the database to get human observation
	ros::Subscriber fact_subscriber_; //used if we are looping on a topic to get observations

	// std::map<std::string,ros::Publisher> human_action_topics_; //actions performed by each humans are published here
	ros::Publisher human_executable_actions_pub_;

	//a std::map that links an action name with a service to get its preconditions\postconditions
	std::map<std::string,ros::ServiceClient> action_preconditions_services_;

	//values read from parameters
	std::vector<std::string> human_list_;
	std::vector<std::string> actions_to_monitor_;  
	std::vector<std::string> object_list_;
	std::map<std::string,std::vector<std::string> > object_affordances_; //links an object to possible actions

	boost::mutex mutex_human_objects_;
	std::map<std::string,std::string> human_objects_;  //std::maps each human to an object that he holds

	std::string robot_name_; 

	bool use_database_; //parameter that decides if we use the db or a topic to get observations
	std::map<std::string,std::string> action_targets_;

};

#endif