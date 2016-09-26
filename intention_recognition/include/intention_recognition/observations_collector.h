#ifndef OBSERVATIONS_COLLECTOR_H
#define OBSERVATIONS_COLLECTOR_H

#include <ros/ros.h>
#include <IntentionGraph.h>
#include <Mdp.h>
#include <situation_assessment_msgs/Fact.h>
#include <situation_assessment_msgs/QueryDatabase.h>
#include <intention_recognition/string_operations.h>

class ObservationsCollector {
public:
	ObservationsCollector(ros::NodeHandle node_handle);

	//gets the initial state of an MDP
	std::map<std::string,std::string> getInitialState(std::string agent, std::vector<Mdp*> mdps);
	//gets the current evidence for an IG
	std::map<std::string,std::string> getEvidence(std::string agent, IntentionGraph* ig);
	
	//utility that queries the database and returns a value
	string queryDatabase(situation_assessment_msgs::Fact f); 
	std::vector<std::string> queryDatabaseVector(situation_assessment_msgs::Fact f);

private:
	ros::NodeHandle node_handle_;
	ros::ServiceClient database_service_;
	std::string robot_name_;
	std::map<std::string,int> depth_areas_;

};

#endif