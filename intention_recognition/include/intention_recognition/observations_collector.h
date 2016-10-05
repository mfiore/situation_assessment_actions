#ifndef OBSERVATIONS_COLLECTOR_H
#define OBSERVATIONS_COLLECTOR_H

#include <ros/ros.h>
#include <IntentionGraph.h>
#include <Mdp.h>
#include <VariableSet.h>
#include <situation_assessment_msgs/Fact.h>
#include <situation_assessment_msgs/QueryDatabase.h>
#include <situation_assessment_msgs/QueryDatabase.h>
#include <intention_recognition/string_operations.h>

class ObservationsCollector {
public:
	ObservationsCollector(ros::NodeHandle node_handle);

	//gets the initial state of an MDP
	VariableSet getInitialState(std::string agent, std::vector<Mdp*> mdps);
	//gets the current evidence for an IG
	VariableSet getEvidence(std::string agent, IntentionGraph* ig);
	
	std::string getAt(std::string agent);

	//utility that queries the database and returns a value
	string queryDatabase(situation_assessment_msgs::Fact f); 
	std::vector<std::string> queryDatabaseVector(situation_assessment_msgs::Fact f);

private:
	ros::NodeHandle node_handle_;
	ros::ServiceClient database_service_;
	std::string robot_name_;
	double reach_,close_,medium_,far_;

};

#endif