#include "intention_recognition/intention_recognition.h"


IntentionRecognition::IntentionRecognition(ros::NodeHandle node_handle):node_handle_(node_handle),observations_collector_(node_handle) {

	node_handle_.getParam("/situation_assessment/intention_recognition/contexts",contexts_);
	ROS_INFO("INTENTION_RECOGNITION - context list:");
	for (string c:contexts_) {
		ROS_INFO("INTENTION_RECOGNITION - %s",c.c_str());
	}
	node_handle_.getParam("/situation_assessment/intention_recognition/intention_list",intention_list_);
	for (std::string i : intention_list_) {
		ROS_INFO("INTENTION_RECOGNITION - intention %s",i.c_str());
		std::string mdp_path;
		node_handle_.getParam("/situation_assessment/intention_recognition/intention_details/"+i+"/mdp_path",mdp_path);

		ROS_INFO("INTENTION_RECOGNITION - mdp_path is %s",mdp_path.c_str());

		Mdp intention_mdp;
		bool ok=intention_mdp.readMdp(mdp_path);
		if (!ok) {
			ROS_WARN("INTENTION_RECOGNITION - error reading mdp for intention %s and path %s",i.c_str(),mdp_path.c_str());	
		}
		else {
			ROS_INFO("INTENTION_RECOGNITION - Adding MDP for intention %s with path %s",i.c_str(),mdp_path.c_str());
			mdp_map_[i]=&intention_mdp;
		}

		StringVector contexts;
		node_handle_.getParam("/situation_assessment/intention_recognition/intention_details/"+i+"/linked_contexts",
			contexts);
		linked_contexts_[i]=contexts;
		for (std::string c:contexts) {
			ROS_INFO("INTENTION_RECOGNITION - context %s",c.c_str());
			double influence;
			node_handle_.getParam("/situation_assessment/intention_recognition/intention_details/"+i+
				"/context_probabilities/"+c,influence);

			ROS_INFO("INTENTION_RECOGNITION - influence is %f",influence);

			StringPair p;
			p.first=i;
			p.second=c;
			intention_conditional_probabilities_[p]=influence;
		}
	}
	action_sub_=node_handle.subscribe("/situation_assessment/human_executable_actions",1000,
		&IntentionRecognition::actionCallback,this);

	
	pub_intentions_=node_handle.advertise<situation_assessment_actions_msgs::IntentionGraphResult>(
		"situation_assessment/agents_inference",1000);

	ROS_INFO("INTENTION_RECOGNITION - running");
	
	// ros::ServiceServer start_monitor_server=node_handle.advertiseService("situation_assessment/monitor_intentions",startMonitoring);
	// ros::ServiceServer stop_monitor_server=node_handle.advertiseService("situation_assessment/stop_monitor_intentions",stopMonitoring);

	// ROS_INFO("INTENTION_RECOGNITION - advertising services");


}



void IntentionRecognition::createIntentionGraph(std::string agent, std::vector<std::string> actions) {
		boost::lock_guard<boost::mutex> lock(mutex_igs_);


	    std::vector<IntentionNode> intention_nodes;
	    std::vector<Mdp*> mdps;

	    std::map<std::string,std::string> mdp_parameters;
	    mdp_parameters["agent"]=agent;

	    //create intention nodes and mdp vector
	    for (std::string intention:intention_list_) {
	    	if (mdp_map_.find(intention)==mdp_map_.end()) {
	    		ROS_WARN("INTENTION_RECOGNITION - intention %s has no associated mdp",intention.c_str());
	    	}

	    	IntentionNode i;
	    	i.name=intention;
	    	i.linked_contexts=linked_contexts_[intention];
	    	for (std::string context:i.linked_contexts) {
	    		std::pair<std::string,std::string> map_input {intention,context};
	    		i.influence.push_back(intention_conditional_probabilities_[map_input]);	
	    	}

	    	intention_nodes.push_back(i);

	    	Mdp* m=mdp_map_.at(intention);
	    	m->assignParameters(mdp_parameters);

	    	//check if every intention is present

			mdps.push_back(m);   	
	    }   
	    std::map<std::string,std::string> initial_state=observations_collector_.getInitialState(agent,mdps);
	    IntentionGraph ig;
	    ig.setGraph(contexts_,intention_nodes,actions,mdps,initial_state);
	    agents_intentions_[agent]=&ig;
}


//creates an IG for an agent
bool IntentionRecognition::startMonitoring(situation_assessment_actions_msgs::StartMonitorIntentions::Request &req,
	situation_assessment_actions_msgs::StartMonitorIntentions::Response &res) {


}

std::string IntentionRecognition::createActionString(action_management_msgs::Action a) {
	std::string action_string;
	std::string agent="";
	std::string target="";
	std::string object="";
	std::vector<common_msgs::Parameter> parameters=a.parameters; 
	for (int i=0; i<parameters.size(); i++) {
		if (parameters[i].name=="main_agent") {
			agent=parameters[i].value;
		}
		else if (parameters[i].name=="main_object") {
			object=parameters[i].value;
		}
		else if (parameters[i].name=="target") {
			target=parameters[i].value;
		}
	}
	if (target==""){
		action_string=agent+"_"+a.name+"_"+object;
	}
	else if (object=="") {
		action_string=agent+"_"+a.name+"_"+target;	
	}
	else {
		action_string=agent+"_"+a.name+"_"+object+"_"+target;	
	}
	return action_string;
}

//tips to make it better:
// publishing executable actions only when there is a difference
// linking a code to the message to say if its difference
// publishing another topic to inform of the difference
// making this a service
void IntentionRecognition::actionCallback(const situation_assessment_actions_msgs::ExecutableActions::ConstPtr& msg) {
	
	std::map<std::string,std::vector<action_management_msgs::Action> > agents_to_recreate; 
	for (int i=0;i<msg->executable_agents_actions.size();i++) {
		string agent=msg->executable_agents_actions[i].agent;
		std::vector<action_management_msgs::Action> new_actions=msg->executable_agents_actions[i].actions;
		if (executable_actions_.find(agent)==executable_actions_.end()) {
			executable_actions_[agent]=new_actions;
		}
		else {
			std::vector<action_management_msgs::Action> old_actions=executable_actions_.at(agent);
			if (new_actions.size()!=old_actions.size()) {
				agents_to_recreate[agent]=new_actions;
			}			
			else {
				bool found_difference=false;
				int j=0;
				while (j<new_actions.size() && !found_difference) {
					if (old_actions[j].name!=new_actions[j].name) {
						found_difference=true;
						break;
					}
					else {
						std::vector<common_msgs::Parameter> old_parameters=old_actions[j].parameters;
						std::vector<common_msgs::Parameter> new_parameters=new_actions[j].parameters;
						if (old_parameters.size()!=new_parameters.size()) {
							found_difference=true;
							break;
						}
						else {
							for (int k=0;k<old_parameters.size();k++) {
								if (old_parameters[k].name!=new_parameters[k].name ||
									old_parameters[k].value!=new_parameters[k].value) {
									found_difference=true;
								break;
								}
							}
						}
					}
					j++;		
				}
				if (found_difference) {
					agents_to_recreate[agent]=new_actions;
				}
			}
		}
	}
 	for (auto a:agents_to_recreate) {
 		std::vector<std::string> action_strings;
 		for (int i=0;i<a.second.size();i++) {
 			action_strings.push_back(createActionString(a.second[i]));
 		}
 		createIntentionGraph(a.first,action_strings);
 	}
}


void IntentionRecognition::intentionLoop(){
	ros::Rate r(3);

	while (ros::ok()) {
		//this is the main loop, which gets evidence for all the currently loaded IGs, computes
		//their probabilities, publish them, and then checks for new requests.
		situation_assessment_actions_msgs::IntentionGraphResult ig_result_message;
		std::map<std::string,IntentionGraph*> intention_graphs=getIntentionGraphs();
		for (auto ig: intention_graphs) {
			situation_assessment_actions_msgs::AgentInference agent_inference_msg;
			std::map<std::string,std::string> evidence=observations_collector_.getEvidence(ig.first,ig.second);
			std::map<std::string,double> agent_probability=ig.second->computeProbability(evidence);
			
			std::vector<string> agent_actions=ig.second->getActionNodes();
			agent_inference_msg.actions=agent_actions;
			for (std::string a:agent_actions) {
				agent_inference_msg.actions_prob.push_back(agent_probability.at(a));
			}
			agent_inference_msg.intentions=intention_list_;
			for (std::string i:intention_list_) {
				agent_inference_msg.intentions_prob.push_back(agent_probability.at(i));
			}
			ig_result_message.agents_inference.push_back(agent_inference_msg);
		}
		pub_intentions_.publish(ig_result_message);
		r.sleep();
	}
	ROS_INFO("INTENTION_RECOGNITION - killing node");
	for (auto m:mdp_map_) {
		delete m.second;
	}
}

std::map<std::string,IntentionGraph*> IntentionRecognition::getIntentionGraphs() {
	boost::lock_guard<boost::mutex> lock(mutex_igs_);

	return agents_intentions_;
}
