#include <ros/ros.h>
#include <iostream>
#include "IntentionGraph.h"
#include "Mdp.h"
#include "situation_assessment_actions_msgs/AgentInference.h"
#include "situation_assessment_actions_msgs/IntentionGraphResult.h"
#include "situation_assessment_actions_msgs/StartMonitorIntentions.h"
#include "situation_assessment_actions_msgs/StopMonitorIntentions.h"
#include <boost/thread/mutex.h>
#include <utility>


std::map<std::string,IntentionGraph*> agents_intentions_;
boost::mutex mutex_ig_;

std::map<std::string,std::vector<std::string> > agent_actions_;  //actions to monitor for each agent
std::map<std::string,std::vector<std::string> > agent_intentions_;    //intentions to monitor for each agent
std::map<std::string,std::map<std::string,std::string> > agent_contexts_; //context and values assigned for each agent


std::map<std::string,std::string> intention_path_; //paths to the mdps of each intention
std::map<std::string,std::vector<std::string> > linked_contexts_; //contexts linked to each intention
std::map<std::pair<std::string,std::string>,double > intention_conditional_probabilities_: //P(I|C) table


std::map<std::string,Mdp*> mdp_map_;


bool startMonitoring(situation_assessment_actions_msgs::StartMonitorIntentions::Request &req,
	situation_assessment_actions_msgs::StartMonitorIntentions::Response &res) {

	ROS_INFO("INTENTION_RECOGNITION - Received request to monitor %s",req.agent.c_str());

    std::vector<string> contexts = req.contexts;
    std::map<std::string,std::string> context_map;

    int i=0;
    for (std::string c:contexts) {
    	context_map[c]=req.context_values[i];
    	i++;
    }

    std::vector<IntentionNode> intention_nodes;
    std::vector<Mdp*> mdps;

    for (std::string intention:req.intentions) {
    	IntentionNode i;
    	i.name=intention;
    	i.linked_contexts=linked_contexts[i];
    	for (std::string context:linked_contexts[i]) {
    		std::pair<std::string,std::string> map_input {intention,context};
    		i.influence.push_back(intention_conditional_probabilities_[map_input]);	
    	}

    	intention_nodes_.push_back(i);

  //   	Mdp *m;
		// if (mdp_map_.find(intention)==mdp_map_.end()) {
		// 	m=new Mdp();
		// 	m->create(intention_path_.at(i),false);
		// 	mdp_map_[intention]=&m;
		// }
		// else {
		// 	m=mdp_map_.at(intention);
		// }
		// setInitialState(m);
		// mdps.push_back(m);   	
    }

    ig.setGraph(contexts,intentions,actions,mdps,initial_state);
    
    
	}
}

map<string,string> getEvidence() {
	std::vector<string> actions={"agent_move_table","agent_move_kitchen"};

    VariableSet evidence;
    evidence.set["distance_human_move_table"]="close";
    evidence.set["toward_human_move_table"]="t";
    evidence.set["distance_human_move_kitchen"]="far";
    evidence.set["toward_human_move_kitchen"]="f";



	return map<string,string>();
}

int main(int argc, char ** argv) {
	ros::init(argc,argv,"situation_assessment/intention_recognition");

	ROS_INFO("INTENTION_RECOGNITION - started node");

	ros::NodeHandle node_handle;

	
	ros::ServiceServer start_monitor_server=node_handle.advertiseService("situation_assessment/monitor_intentions",startMonitoring);
	ros::ServiceServer stop_monitor_server=node_handle.advertiseService("situation_assessment/stop_monitor_intentions",stopMonitoring);

	ROS_INFO("INTENTION_RECOGNITION - advertising services");



	ros::Publisher pub_intentions;
	pub=node_handle.advertise<situation_assessment_actions_msgs::IntentionGraphResult>("situation_assessment/agents_inference",1000);
		ros::Rate r(3);

	ROS_INFO("INTENTION_RECOGNITION - running");

	while (ros::ok()) {
		// mutex_ig_.lock();

		situation_assessment_actions_msgs::IntentionGraphResult ig_result_message;
		for (auto ig: agents_intentions_) {
			situation_assessment_actions_msgs::AgentInference agent_inference_msg;
			std::map<std::string,std::string> evidence=getEvidence();
			std::map<std::string,double> agent_probability=ig.computeProbability(evidence);
			
			agent_inference_msg.actions=agent_actions_[ig.first];
			for (std::string a:agent_actions_[ig.first]) {
				agent_inference_msg.actions_prob.push_back(agent_probability.at(a));
			}
			agent_inference_msg.intentions=agent_intentions_[ig.first];
			for (std::string i:agent_intentions_[ig.first]) {
				agent_inference_msg.intentions_prob.push_back(agent_probability.at(i));
			}
			ig_result_message.agents_inference.push_back(agent_inference_msg);
		}
		pub.publish(ig_result_message);
		// mutex_ig_.unlock();
		r.sleep();
		ros::spinOnce(); //update graphs
	}
	ROS_INFO("INTENTION_RECOGNITION - killing node");
	for (auto m:mdp_map_) {
		delete m.second;
	}


	return 0;
}