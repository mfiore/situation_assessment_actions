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

		Mdp* intention_mdp=new Mdp();
		bool ok=intention_mdp->readMdp(mdp_path);
		if (!ok) {
			ROS_WARN("INTENTION_RECOGNITION - error reading mdp for intention %s and path %s",i.c_str(),mdp_path.c_str());	
		}
		else {
			ROS_INFO("INTENTION_RECOGNITION - Adding MDP for intention %s with path %s",i.c_str(),mdp_path.c_str());
			mdp_map_[i]=intention_mdp;

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
	node_handle.getParam("/situation_assessment/human_names",humans_);
	// for (string h: humans_) {
	// 	std::vector<string> human_move_actions;
	// 	for (string l:locations_) {
	// 		if (l!="this") {
	// 			human_move_actions.push_back(h+"_move_"+l);
	// 		}
	// 	}
	// 	move_actions_[h]=human_move_actions;
	// }

	action_sub_=node_handle.subscribe("/situation_assessment/human_executable_actions",1000,
		&IntentionRecognition::actionCallback,this);
	ros::Rate r(3);
	ROS_INFO("INTENTION_RECOGNITION - waiting for executable actions");
	while (action_sub_.getNumPublishers()==0) {
		r.sleep();
	}


	
	pub_intentions_=node_handle.advertise<situation_assessment_actions_msgs::IntentionGraphResult>(
		"situation_assessment/agents_inference",1000);

	for (string i:intention_list_) {
		string namepub=i;
		ros::Publisher* pub=new ros::Publisher();
		*pub=node_handle.advertise<situation_assessment_actions_msgs::IntentionProb>(
			namepub,1000);
		pub_intentions_plot_[i]=pub;
	}
	// tea_pub_=node_handle_.advertise<situation_assessment_actions_msgs::IntentionProb>("intention_prob",1000);
	ROS_INFO("INTENTION_RECOGNITION - running");
	
	// ros::ServiceServer start_monitor_server=node_handle.advertiseService("situation_assessment/monitor_intentions",startMonitoring);
	// ros::ServiceServer stop_monitor_server=node_handle.advertiseService("situation_assessment/stop_monitor_intentions",stopMonitoring);

	// ROS_INFO("INTENTION_RECOGNITION - advertising services");

	ros::Duration(5).sleep();
}



void IntentionRecognition::createIntentionGraph(std::string agent, std::vector<std::string> actions) {
		boost::lock_guard<boost::mutex> lock(mutex_igs_);

		ROS_INFO("INTENTION_RECOGNITION creating intention graph for agent %s",agent.c_str());
	    std::vector<IntentionNode> intention_nodes;
	    std::vector<Mdp*> mdps;

	    std::map<std::string,std::string> mdp_parameters;
	    mdp_parameters["agent"]=agent;

	    //create intention nodes and mdp vector
	    for (std::string intention:intention_list_) {
	    	if (mdp_map_.find(intention)==mdp_map_.end()) {
	    		ROS_WARN("INTENTION_RECOGNITION - intention %s has no associated mdp",intention.c_str());
	    	}
	    	Mdp* m=mdp_map_.at(intention);
			// m->printStates();

	    	m->assignParameters(mdp_parameters);

	    	IntentionNode i;
	    	i.name=intention;
	    	i.linked_contexts=linked_contexts_.at(intention);
	    	ROS_INFO("INTENTION_RECOGNITION assigned linked contexts");
	    	for (std::string context:i.linked_contexts) {
	    		std::pair<std::string,std::string> map_input {intention,context};
	    		i.influence.push_back(intention_conditional_probabilities_[map_input]);	
	    	}

	    	intention_nodes.push_back(i);

	    	//check if every intention is present

			mdps.push_back(m);   	
	    }   
	    VariableSet initial_state=observations_collector_.getInitialState(agent,mdps);
	    // cout<<"Initial State\n";
	    // cout<<initial_state.toString()<<"\n\n";
	    // ROS_INFO("INTENTION_RECOGNITION got initial state");
	    IntentionGraph* ig;
	    if (agents_intentions_.find(agent)==agents_intentions_.end()) {
	    	ig=new IntentionGraph();
	    	agents_intentions_[agent]=ig;
	    }
	    else {
	    	ig=agents_intentions_.at(agent);
	    }
	    // for (std::string c:contexts_) {
	    // 	ROS_INFO("%s",c.c_str());
	    // }
	    // for (std::string a:actions){
	    // 	ROS_INFO("%s",a.c_str());
	    // }
	    // ROS_INFO("%ld",intention_nodes.size());

	    ig->setGraph(contexts_,intention_nodes,actions,mdps,initial_state);
	    ROS_INFO("INTENTION_RECOGNITION set graph");
	    // if (agents_intentions_.find(agent)!=agents_intentions_.end()) {
	    	// delete agents_intentions_.at(agent);
	    // }
	    // agents_intentions_[agent]=ig;
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
	// ROS_INFO("INTENTION_RECOGNITION in action callback");
	// ROS_INFO("In callback action");

	std::set<string> agents_to_recreate;
	std::map<std::string,bool> found_actions;
	for (string agent:humans_) {
	 // //check if the location of the agent has changed
		// string location=observations_collector_.getAt(agent);
		// if (human_locations_.find(agent)==human_locations_.end()) {
		// 	human_locations_[agent]=location;
		// 	agents_to_recreate.insert(agent);
		// }
		// else if (human_locations_.at(agent)!=location) {
		// 	human_locations_[agent]=location;
		// 	agents_to_recreate.insert(agent);
		// }
		found_actions[agent]=false;
	}
	// ROS_INFO("message size %ld",msg->executable_agents_actions.size());


	for (int i=0;i<msg->executable_agents_actions.size();i++) {
		string agent=msg->executable_agents_actions[i].agent;
		std::vector<action_management_msgs::Action> new_actions=msg->executable_agents_actions[i].actions;

		// ROS_INFO("INTENTION_RECOGNITION in loop");
		found_actions[agent]=true;
		if (agents_to_recreate.find(agent)!=agents_to_recreate.end()) {
			executable_actions_[agent]=new_actions;
		}
		else {
			// ROS_INFO("INTENTION_RECOGNITION in else");
			if (executable_actions_.find(agent)==executable_actions_.end()) {
				executable_actions_[agent]=new_actions;
				agents_to_recreate.insert(agent);
			}
			else {
				std::vector<action_management_msgs::Action> old_actions=executable_actions_.at(agent);
				if (new_actions.size()!=old_actions.size()) {
					// ROS_INFO("INTENTION_RECOGNITION different size");
					executable_actions_[agent]=new_actions;
					agents_to_recreate.insert(agent);
				}			
				else {
					// ROS_INFO("INTENTION_RECOGNITION same size");
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
						executable_actions_[agent]=new_actions;
						agents_to_recreate.insert(agent);
					}
				}
			}

		}
	}
	for (string agent:humans_) {
		if (found_actions.at(agent)==false) {
			executable_actions_.erase(agent);
		}
	}

	// ROS_INFO("INTENTION_RECOGNITION agents to recreate size is %ld",agents_to_recreate.size());
 	for (string agent:agents_to_recreate) {
 		// ROS_INFO("In agent to recreate");
 		std::vector<std::string> action_strings;
 		// string location=human_locations_.at(agent);
 		// for (string l:locations_) {
 		// 	if (l!=location && l!="this") {
 		// 		action_strings.push_back(agent+"_move_"+l);
 		// 	}
 		// }
 		// action_strings.push_back(agent+"_move_sofa");

 		if (executable_actions_.find(agent)!=executable_actions_.end()) {
	 		std::vector<action_management_msgs::Action> agent_actions=executable_actions_.at(agent);
	 		for (int i=0;i<agent_actions.size();i++) {
	 			std::string action_s=createActionString(agent_actions[i]);
	 			// ROS_INFO("INTENTION_RECOGNITION action string is %s",action_s.c_str());
	 			action_strings.push_back(action_s);
	 		}
 		}
 		// ROS_INFO("action sstrings");
 		// for (string s:action_strings) {
 		// 	ROS_INFO("%s",s.c_str());
 		// }
 		ROS_INFO("INTENTION_RECOGNITION creating intention graph");
 		createIntentionGraph(agent,action_strings);
 		ROS_INFO("INTENTION_RECOGNITION done");

 	}
}


void IntentionRecognition::intentionLoop(){
	ros::Rate r(3);

	while (ros::ok()) {
		ros::spinOnce();
		//this is the main loop, which gets evidence for all the currently loaded IGs, computes
		//their probabilities, publish them, and then checks for new requests.
		situation_assessment_actions_msgs::IntentionGraphResult ig_result_message;
		std::map<std::string,IntentionGraph*> intention_graphs=getIntentionGraphs();
		std::map<std::string,situation_assessment_actions_msgs::IntentionProb> plot_msg;
		for (auto ig: intention_graphs) {
			situation_assessment_actions_msgs::AgentInference agent_inference_msg;
			VariableSet evidence=observations_collector_.getEvidence(ig.first,ig.second);
			// ROS_INFO("%s",evidence.toString().c_str());
			// ROS_INFO("Before computing probabilities");
			std::map<std::string,double> agent_probability=ig.second->computeProbability(evidence);
			// ROS_INFO("After computing probabilities");
			// for (auto p:agent_probability) {
			// 	ROS_INFO("%s %f",p.first.c_str(),p.second);
			// }
			
			std::vector<string> agent_actions=ig.second->getActionNodes();
			agent_inference_msg.actions=agent_actions;
			for (std::string a:agent_actions) {
				// ROS_INFO("%s %f",a.c_str(),agent_probability.at(a));
				agent_inference_msg.actions_prob.push_back(agent_probability.at(a));
			}
			agent_inference_msg.intentions=intention_list_;
			for (std::string i:intention_list_) {
				agent_inference_msg.intentions_prob.push_back(agent_probability.at(i));
				situation_assessment_actions_msgs::IntentionProb new_msg;
				new_msg.prob=agent_probability.at(i);
				plot_msg[i]=new_msg;

			}
			ig_result_message.agents_inference.push_back(agent_inference_msg);
		}
		pub_intentions_.publish(ig_result_message);
		// situation_assessment_actions_msgs::IntentionProb msg_prob;
		// msg_prob.prob=0.9;
		// tea_pub_.publish(msg_prob);
		// plot_msg.at("drink_tea").data=1.0;
		// tea_pub_.publish(plot_msg.at("drink_tea"));
		for (string i:intention_list_) {
			pub_intentions_plot_.at(i)->publish(plot_msg.at(i));
		}
		// r.sleep();
	}
	// ROS_INFO("INTENTION_RECOGNITION - killing node");
	for (auto m:mdp_map_) {
		delete m.second;
	}
	std::map<std::string,IntentionGraph*> intention_graphs=getIntentionGraphs();
	for (auto ig :intention_graphs) {
		delete ig.second;
	}
}


std::map<std::string,IntentionGraph*> IntentionRecognition::getIntentionGraphs() {
	boost::lock_guard<boost::mutex> lock(mutex_igs_);

	return agents_intentions_;
}
