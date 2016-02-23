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
	void monitorLoop();
	void databaseLoop();

	ros::NodeHandle node_handle_;
	ros::ServiceClient database_query_client_;
	ros::Subscriber fact_subscriber_;

	map<string,ros::Publisher> human_action_topics_;
	map<string,ros::ServiceClient> action_preconditions_services_;
	map<string,ros::ServiceClient> action_postconditions_services_;
	map<string,string> object_to_actions_;


	vector<string> human_list_;
	vector<string> actions_to_monitor_;
	vector<string> object_list_;
	map<string,vector<string> > object_affordances_;

	vector<string> monitored_actions_;

	void agentFactCallback(const situation_assessment_msgs::FactList::ConstPtr& msg);

	string getHumanObject(string human);
	void setHumanObject(string human,string object);
	double getObjectDistance(string human,string object);
	void setObjectDistance(string human,string object,double distance);

	boost::mutex mutex_human_objects_;
	map<string,string> human_objects_;
	boost::mutex mutex_object_distance_;
	map<pair<string,string>,double> object_distances_;

	double trigger_distance_;

	string robot_name_;

	bool use_database_;

	ros::ServiceClient database_client_;
};

//creating a topic for each agent with the performed actions

//action list
//including the action library?

#endif