#include <intention_recognition/intention_recognition.h>

IntentionRecognition::IntentionRecognition(string human, vector<string> context_list, vector<string> intention_list, vector<string> action_list, 
		vector<Intention> intentions_, vector<string> target_list, map<string,vector<string> > target_action_list, 
		ros::NodeHandle node_handle_):
human_(human),intentions_(intentions), context_list_(context_list), intention_list_(intention_list),action_list_(action_list),intentions_(intentions),
 BayesianNetwork(0), target_list_(target_list),target_action_list_(target_action_list) node_handle_(node_handle) {

	distance_values_.push_back("out");
	distance_values_.push_back("far");
	distance_values_.push_back("medium");
	distance_values_.push_back("close");
	distance_values_.push_back("reach");	

	intention_values_.push_back("t");
	intention_values_.push_back("f");

	for (string a:action_list_) {
		ros::ServiceClient client_preconditions=node_handle_.serviceClient<action_management_msgs::CheckPreconditions>("/supervision/actions/"+actions_to_monitor_[i]+"/check_preconditions");
		client_preconditions_services_[a]=client_preconditions;
	}
	database_client_=node_handle_.serviceClient<situation_assessment_msgs::QueryDatabase>("/situation_assessment/query_database");

	publish_intention_=node_handle_.advertise<situation_assessment_action_msgs::AgentInference("/situation_assessment/agent_inference/"+human_,1000);
}





string IntentionRecognition::getHumanObject() {
	situation_assessment_msgs::QueryDatabase srv;
	srv.fact.model=robot_name_;
	srv.fact.subject=human_;
	srv.fact.predicate.push_back("has");

	string object="";
	if (database_client_.call(srv)) {
		if (srv.result.result.size()>0) {
			if (srv.result.result[0].value.size()>0) {
				object=srv.result.result[0].value[0];
			}
		}
	}
	return object;
}

void IntentionRecognition::checkValidActions() {
	valid_actions_.clear();
	for (string target: target_list_) {
		vector<string> target_actions=target_action_list_[target];
		for (string action: target_Actions) {
			common_msgs::ParameterList parameter_list;
			common_msgs::Parameter target_parameter;

			target_parameter.name="target";
			target_parameter.value=target;
			parameter_list.parameter_list.push_back(target_parameter);
			string human_object=getHumanObject();
			if (human_object!="") {
				common_msgs::Parameter object_parameter;
				object_parameter.name="object";
				object_parameter.value=human_object;
				parameter_list.parameter_list.push_back(object_parameter);
			}
			action_management_msgs::CheckPreconditions preconditions_msg;
			preconditions_msg.request.parameters=parameter_list;

			if (action_preconditions_services_[action].call(preconditions_msg)) {
				if (preconditions_msg.response.value==true) {
					valid_actions_.push_back(action);
				}
			}	
			else {
				ROS_ERROR("ACTION_MONITORS failed to check preconditions for action %s",action.c_str());
			}
		}
	}
}
				
void IntentionRecognition::createNetwork() {

	checkValidActions();

	bayes.setMpdes(context_list.size()+intentions.size()+valid_actions_.size()*4);

	createContextNodes();
	createIntentionNodes();
	createActionNodes();
	bayes.updateJoinTree();
}
void IntentionRecognition::createContextNodes() {
	for (string c:context_list_) {
		bayes.addNode(c,std::vector<string>(),false,"uniform","");
	}
}
void IntentionRecognition::createIntentionNodes() {
	for (Intention i:intentions_) {
		bayes.addNode(i.name_,i.contexts_,false,"or","");
	}
}


void IntentionRecognition::loadMdps() {
	for (Intention i:intentions_) {
		ros::ServiceClient mdp_client=node_handle_.serviceClient<mdp_msgs::GetActionValue>("planning/mdp/"+i.name_/get_action_value);
		mdp_client.waitForExistence();
		mdp_action_value_services_[i.name]=mdp_client;
	}
}

double getQValue(string intention, string action) {
	mdp_msgs::GetActionValue mdp_srv;
	mdp_action_value_services_[intention].call(mdp_srv);
	return  mdp_srv.result.value;
}


void IntentionRecognition::createActionNodes() {
	std::vector<std::vector<int> > table_variables;
	for (Intention intention : intentions_) {
	    std::map<string, int> node_values = bayes->getValues(intention);
	    std::vector<int> values;
	    for (int i = 0; i < node_values.size(); i++) {
	        values.push_back(i);
	    }
	    table_variables.push_back(values);
	}
	//get the matrix corresponding to the intentions combination table
	std::vector<std::vector<int> > table_assignments;
	NestedLoop<int> loop(table_variables);
	table_assignments = loop.buildMatrix();

	std::vector<double> sumq(intentions_.size(), 0);
	for (string a : action_list_) {
		for (int i=0; i<intention_list_.size();i++) {

	        sumq[i] = sumq[i] + getQValue(intention_list[i],a);
	    }
	}
	for (string a : actions) {
	    std::vector<prob_assignment> prob_table;
	    bool first_row = true;
	    for (auto row : table_assignments) {
	        prob_assignment prob_row;
	        double qrow = 0;
	        double tot_sum = 0;
	        for (int i = 0; i < intention_list_.size(); i++) {
	            prob_row.parentValues[intentions_list[i]] = intention_values_[row[i]];
	            if (row[i] == 1) {
	                qrow = qrow + getQValue(intention_list_[i],a);
	                tot_sum = tot_sum + sumq[i];
	            }
	        }
	        if (tot_sum == 0) {
	            prob_row.prob = 0;
	        } else {
	            prob_row.prob = qrow / tot_sum;
	        }
	        prob_table.push_back(prob_row);
	    }
	    bayes->addNode(a, intention_list, prob_table);
	}
}

void IntentionRecognition::createObservationNodes() {
	for (string action : action_list_) {
		std::vector<string> parents = {action};
		bayes.laddMultiValueNode("body_distance_" + action, action, distance_values_);
		bayes.addNode("body_toward_" + action, parents, false, "or", "");
		bayes.laddMultiValueNode("hand_distance_" + action, action, distance_values_);
		bayes.addNode("hand_toward_" + action, parents, false, "or", "");
	}
}


bool shouldRun() {
	boost::lock_guard<boost::mutex> l(mutex_should_run_);
	return should_run_;
}

void stop() {
	boost::lock_guard<boost::mutex> l(mutex_should_run_);
	should_run_=false;
}

void IntentionRecognition::start() {
	createNetwork();

	ros::Rate r(3);
	while (shouldRun()) {
		if (newAction()) {
			createNetwork();
		}

		map<string,string> evidence=getEvidence();
		map<string,double> probabilities=bayes->computeProbability(evidence);


		situation_assessment_action_msgs::AgentInference msg;x
		for (string intention: intentions_) {
			msg.intentions.push_back(intention);
			msg.intentions_prob.push_back(probabilities[intention]);
		}
		for (string action:valid_actions_) {
			msg.actions.push_back(action);
			msg.actions_prob.push_back(probabilities[action]);
		}
		publish_intention_.publish(msg);



		r.sleep();
	}

}