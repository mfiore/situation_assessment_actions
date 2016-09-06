/*
author: mfiore
This node is used to give an estimation of the most likelyl intentions and actions of an agent.
External nodes can request inference by using the service:

situation_assessment/monitor_intentions  of type situation_assessment_msgs::StartMonitorIntentions
if the agent in this message is new, the node will create a new IG, if not it will update the previous IG of the
agent with the new information

situation_assessment/stop_monitor_intentions stops the inference process of an agent

it's important to specify in the situation_assessment_actions.yaml msg the link between each intention and the mdps (if mdps are used)
if there is no mdp the system can't infer action and should receive only actions in the message

 */

#include <ros/ros.h>
#include <iostream>
#include "IntentionGraph.h"
#include "Mdp.h"
#include "situation_assessment_actions_msgs/AgentInference.h"
#include "situation_assessment_actions_msgs/IntentionGraphResult.h"
#include "situation_assessment_actions_msgs/StartMonitorIntentions.h"
#include "situation_assessment_actions_msgs/StopMonitorIntentions.h"
#include "situation_assessment_msgs/QueryDatabase.h"
#include "situation_assessment_msgs/Fact.h"
#include <boost/thread/mutex.h>
#include <utility>

//steps
//- recognizing intentions with action stream
//- recognizing actions
//- correcting divergent belief


std::map<std::string,IntentionGraph*> agents_intentions_; //links an Intention Graph to each agent

std::map<std::string,std::vector<std::string> > agent_actions_;  //actions to monitor for each agent
std::map<std::string,std::vector<std::string> > agent_intentions_;    //intentions to monitor for each agent
std::map<std::string,std::map<std::string,std::string> > agent_contexts_; //context and values assigned for each agent


std::map<std::string,std::vector<std::string> > linked_contexts_; //contexts linked to each intention
std::map<std::pair<std::string,std::string>,double > intention_conditional_probabilities_: //P(I|C) table


std::map<std::string,Mdp*> mdp_map_;

ros::ServiceClient database_service_;

std::string robot_name_;


std::vector<std::string> & stringSplitElems(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

std::vector<std::string> stringSplit(const std::string &s, char delim) {
    std::vector<std::string> elems;
    stringSplitElems(s, delim, elems);
    return elems;
}

bool startMonitoring(situation_assessment_actions_msgs::StartMonitorIntentions::Request &req,
	situation_assessment_actions_msgs::StartMonitorIntentions::Response &res) {

	ROS_INFO("INTENTION_RECOGNITION - Received request to monitor %s",req.agent.c_str());

    std::vector<std::string> contexts = req.contexts;
    std::map<std::string,std::string> context_map;

    int i=0;
    for (std::string c:contexts) {
    	context_map[c]=req.context_values[i];
    	i++;
    }

    std::vector<IntentionNode> intention_nodes;
    std::vector<Mdp*> mdps;

    for (std::string intention:req.intentions) {
    	if (mdp_map_.find(i)==mdp_map_.end()) {
    		ROS_WARN("INTENTION_RECOGNITION - intention %s has no associated mdp",i.c_str());
    	}

    	IntentionNode i;
    	i.name=intention;
    	i.linked_contexts=linked_contexts[i];
    	for (std::string context:linked_contexts[i]) {
    		std::pair<std::string,std::string> map_input {intention,context};
    		i.influence.push_back(intention_conditional_probabilities_[map_input]);	
    	}

    	intention_nodes_.push_back(i);

    	Mdp* m=mdp_map_.at(i);
    	//check if every intention is present

		mdps.push_back(m);   	
    }
    getInitialState(req.agent,mdps)
    ig.setGraph(contexts,intentions,actions,mdps,initial_state);
    
	}
}

std::map<std::string,std::string> getEvidence(std::string agent, IntentionGraph* ig) {
	vector<std::string> observation_nodes=ig->getObservationNodes();
	std::map<std::string,std::string> evidence;

	for (std::string node : observation_nodes) {
		vector<std::string> node_parts=StringOperations::stringSplit(node,'_');
		//two possibilities:
		//- subject action target 
		//- subject action object target
		//our fact will have: 
		//-subject as the agent
		//predicate[0] as node_parts[0] (distance or toward)
		//predicate[1] as node_parts[size-1]					
		
		situation_assessment_msgs::Fact f;
		f.subject=agent;
		f.model=robot_name_;
		f.predicate.push_back(node_parts[0]);
		f.predicate.push_back(node_parts[node_parts.size()-1]);

		evidence[node]=queryDatabase(f)[0];
	}
	return evidence;
}


std::vector<situation_assessment_msgs::Fact> queryDatabase(situation_assessment_msgs::Fact f) {
	situation_assessment_msgs::QueryDatabase srv;
	srv.request.query=f;
	if (database_service_.call(srv)) {
		if (database_service_.response.result.value.size()>0) {
			return database_service_.response.result.value;
		}
		else {
			ROS_WARN("INTENTION_RECOGNITION - did not find values for the %s %s",f.subject.c_str(),f.predicate[0].c_str());
		}
	}
	else {
		ROS_WARN("INTENTION_RECOGNITION - failed to contact database");
	}
	return std::vector<situation_assessment_msgs::Fact>();
}

void getInitialState(std::string agent, vector<Mdp*> mdps) {
	map<std::string,std::string> initial_state;
	for (Mdp* m: mdps) {
		for (std::string v : m->variables) {
			if (initial_state.find(v)==initial_state.end()) {
				situation_assessment_msgs::Fact query;

				vector<std::string> v_parts=StringOperations::stringSplit(v,'_');
				f.model=agent;
				f.subject=v_parts[0];
				for (int i=1; i<v_parts.size();i++)
					f.predicate.push_back(v_parts[i]);
				}
				initial_state[v]=QueryDatabase(f)[0];
			}
		}
	}
	return initial_state;
}

void loadMdp(NodeHandle *node_handle) {
	std::vector<std::string> intention_list;
	node_handle.getParam("/situation_assessment/actions/intention_list",intention_list);
	for (std::string i : intention_list) {
		std::string mdp_path;
		node_handle.getParam("/situation_assessment/actions/intention_list/"+i+"/mdp_path",mdp_path);

		Mdp intention_mdp;
		bool ok=intention_mdp.readMdp(mdp_path);
		if (!ok) {
			ROS_WARN("INTENTION_RECOGNITION - error reading mdp for intention %s and path %s",i.c_str(),mdp_path.c_str());	
		}
		else {
			ROS_INFO("INTENTION_RECOGNITION - Adding MDP for intention %s with path %s",i.c_str(),mdp_path.c_str());
			mdp_map_[i]=*intention_mdp;
		}
	}

}

int main(int argc, char ** argv) {
	ros::init(argc,argv,"situation_assessment/intention_recognition");

	ROS_INFO("INTENTION_RECOGNITION - started node");

	ros::NodeHandle node_handle;

	node_handle.getParam("/robot_name",robot_name_);
	ROS_INFO("INTENTION_RECOGNITION - robot name is %s",robot_name_.c_str());

	ROS_INFO("INTENTION_RECOGNITION - loading MDPs");
	loadMdp(&node_handle);
	ROS_INFO("INTENTION_RECOGNITION - loaded MDPs");
	
	ros::ServiceServer start_monitor_server=node_handle.advertiseService("situation_assessment/monitor_intentions",startMonitoring);
	ros::ServiceServer stop_monitor_server=node_handle.advertiseService("situation_assessment/stop_monitor_intentions",stopMonitoring);

	ROS_INFO("INTENTION_RECOGNITION - advertising services");

	database_service_=node_handle.serviceClient("situation_assessment/query_database");
	ROS_INFO("INTENTION_RECOGNITION - waiting for database");
	database_service_.waitForServer();
	ROS_INFO("INTENTION_RECOGNITION - connected");


	ros::Publisher pub_intentions;
	pub=node_handle.advertise<situation_assessment_actions_msgs::IntentionGraphResult>("situation_assessment/agents_inference",1000);
		ros::Rate r(3);

	ROS_INFO("INTENTION_RECOGNITION - running");

	while (ros::ok()) {

		situation_assessment_actions_msgs::IntentionGraphResult ig_result_message;
		for (auto ig: agents_intentions_) {
			situation_assessment_actions_msgs::AgentInference agent_inference_msg;
			std::map<std::string,std::string> evidence=getEvidence(ig.first,ig.second);
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
		r.sleep();
		ros::spinOnce(); //update graphs. Takes also care of synchronization so there is no need for mutexs
	}
	ROS_INFO("INTENTION_RECOGNITION - killing node");
	for (auto m:mdp_map_) {
		delete m.second;
	}


	return 0;
}