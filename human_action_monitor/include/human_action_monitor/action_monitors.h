/*
This class is used to monitor human actions.
*/

#ifndef ACTION_MONITORS_H
#define ACTION_MONITORS_H
#include <ros/ros.h>
#include <string>
#include <vector>
#include <utility>

#include <action_management_msgs/Action.h>
#include <common_msgs/ParameterList.h>
#include <common_msgs/Parameter.h>
#include <action_management_msgs/CheckPreconditions.h>
#include <action_management_msgs/SetPostconditions.h>

#include <situation_assessment_msgs/QueryDatabase.h>


#include <situation_assessment_msgs/FactList.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/lexical_cast.hpp>



using namespace std;

class ActionMonitors {
public:
	ActionMonitors(ros::NodeHandle node_handle);
	void start();
private:

	double getObjectDistance(string human,string object); //gets the distance between a human's hand and an object
	void setObjectDistance(string human,string object,double distance); //sets this distance

	ros::NodeHandle node_handle_; 
	ros::ServiceClient database_query_client_; //used only if we are using the database to get human observation
	ros::Subscriber fact_subscriber_; //used if we are looping on a topic to get observations

	// map<string,ros::Publisher> human_action_topics_; //actions performed by each humans are published here
	ros::Publisher executed_actions_pub_;

	//a map that links an action name with a service to get its preconditions\postconditions
	map<string,ros::ServiceClient> action_postconditions_services_;

	//values read from parameters
	vector<string> human_list_;
	vector<string> actions_executable_;  
	vector<string> object_list_;
	map<string,vector<string> > object_affordances_; //links an object to possible actions


	boost::mutex mutex_human_objects_;
	map<string,string> human_objects_;  //maps each human to an object that he holds
	boost::mutex mutex_object_distance_;
	map<pair<string,string>,double> object_distances_; //much a pair human-object to a distance

	double trigger_distance_;  //trigger for deciding that an action is done

	string robot_name_; 

	bool use_database_; //parameter that decides if we use the db or a topic to get observations

	ros::ServiceClient database_client_;  //puts facts in the db
};


#endif