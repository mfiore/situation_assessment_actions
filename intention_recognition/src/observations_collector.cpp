#include "intention_recognition/observations_collector.h"

ObservationsCollector::ObservationsCollector(ros::NodeHandle node_handle):node_handle_(node_handle) {
	database_service_=node_handle_.serviceClient<situation_assessment_msgs::QueryDatabase>("situation_assessment/query_database");
	ROS_INFO("INTENTION_RECOGNITION - waiting for database");
	database_service_.waitForExistence();
	ROS_INFO("INTENTION_RECOGNITION - connected");

	node_handle.getParam("/robot_name",robot_name_);

	std::vector<double> distance_threshold;
	node_handle.getParam("/situation_assessment/intention_recognition/distance_threshold",distance_threshold);

	reach_=distance_threshold[0];
	close_=distance_threshold[1];
	medium_=distance_threshold[2];
	far_=distance_threshold[3];

	ROS_INFO("INTENTION_RECOGNITION the thresholds are %f %f %f %f",reach_,close_,medium_,far_);
	std::string reach_,close_,medium_,far_,out_of_range_;


}

//gets the current evidence for an IG
VariableSet ObservationsCollector::getEvidence(std::string agent, IntentionGraph* ig) {
	std::vector<std::string> observation_nodes=ig->getObservationNodes();
	std::map<std::string,std::string> evidence;

	situation_assessment_msgs::Fact is_facing_f;
	is_facing_f.subject=agent+"_head";
	is_facing_f.model=robot_name_;
	is_facing_f.predicate.push_back("isFacing");

	//get the orientation of the agent
	std::vector<string> agent_facing=queryDatabaseVector(is_facing_f);

	for (std::string node : observation_nodes) {
		//observations are in the form observationname_action
		//create a vector from a string by splitting on "_" (e.g. distance_agent_take_mug => [distance,agent,take,mug])
		std::vector<std::string> node_parts=StringOperations::stringSplit(node,'_');
		//two possibilities:
		//- subject action target 
		//- subject action object target
		//our fact will have: 
		//-subject as the agent
		//predicate[0] as node_parts[0] (distance or deltaDistance)
		//predicate[1] as node_parts[size-1] (mug in the ex. above)		

		string action_name=node_parts[2];
		string fact_subject=action_name;
		string observation_type=node_parts[0];
		string target=node_parts[node_parts.size()-1];

		string body_part;

		/*the body part for the query are precomputed. Should be parametrized
		move actions are computed from the torso 
		isFacing from the head
		others from the hand
		*/
		if (action_name=="move") {
			body_part="torso";
		}
		else {
			if (observation_type=="isFacing") {
				body_part="head";
			}
			else {
				body_part="hand";
			}
		}
		situation_assessment_msgs::Fact f;
		
		f.subject=agent+"_"+body_part;
		f.model=robot_name_;
		f.predicate.push_back(observation_type);
		f.predicate.push_back(target);

		std::string actual_value;
		if (node_parts[0]!="isFacing") {
			actual_value=queryDatabase(f);
		}
		if (node_parts[0]=="distance") {
			//distance observations are transformed in reach\close\medium\far from thresholds
			double d=stod(actual_value);
			if (d<reach_) {
				actual_value="reach";
			}
			else if (d<close_) {
				actual_value="close";
			}
			else if (d<medium_) {
				actual_value="medium";
			}
			else if (d<far_) {
				actual_value="far";
			}
		}
		else if (node_parts[0]=="isFacing") {
			if (std::find(agent_facing.begin(),agent_facing.end(),
				node_parts[node_parts.size()-1])!=agent_facing.end()) {
				actual_value="t";
			}
			else {
				actual_value="f";
			}
		}


		evidence[node]=actual_value;
	}
	//now compute the contexts. In the DB contexts are represented as (MODEL AGENT CONTEXT VALUE)
	std::vector<std::string> context_nodes=ig->getContextNodes();
	for (std::string node: context_nodes) {
		situation_assessment_msgs::Fact f;
		f.model=robot_name_;
		f.subject=agent;
		f.predicate.push_back(node);

		evidence[node]=queryDatabase(f);
	}

	VariableSet v;
	v.set=evidence;
	return v;;
}

//gets the initial state of an MDP
VariableSet ObservationsCollector::getInitialState(std::string agent, std::vector<Mdp*> mdps) {
	std::map<std::string,std::string> initial_state; //will contain the state of the system

	situation_assessment_msgs::Fact has_fact;
	has_fact.model=robot_name_;
	has_fact.subject=agent;
	has_fact.predicate={"has"};

	string human_object=queryDatabase(has_fact);

	for (Mdp* m: mdps) {
		std::vector<std::string> original_vars=m->getOriginalVars();
		for (std::string v : original_vars) {
			if (initial_state.find(v)==initial_state.end()) {
				//different mdps can share variables, so we check if this is a new variable

				//split the variable on _
				std::vector<std::string> v_parts=StringOperations::stringSplit(v,'_');
				situation_assessment_msgs::Fact f;
				f.model=robot_name_;

				//for "isAt" we will do a query based on the torso location
				if (v_parts[0]==agent && v_parts[1]=="isAt") {
					f.model=robot_name_;
					f.subject=agent+"_torso";
				}
				else {
					//else subject is the first part of the string (e.g. mug_isAt => subject is mug)
					f.subject=v_parts[0]; 
				}
				if (v_parts[1]=="isAt" && v_parts[0]==human_object) {
						//if the human has an object its location is "human" in our formalism
						initial_state[v]=agent;
					
				}
				else {
					for (int i=1; i<v_parts.size();i++){
						f.predicate.push_back(v_parts[i]);
					}
					initial_state[v]=queryDatabase(f);
				}
			}
		}
	}
	VariableSet v;
	v.set=initial_state;
	return v;
}



//utility that queries the database and returns a value
string ObservationsCollector::queryDatabase(situation_assessment_msgs::Fact f) {
	situation_assessment_msgs::QueryDatabase srv;
	srv.request.query=f;
	if (database_service_.call(srv)) {
		if (srv.response.result.size()>0) {
			if (srv.response.result[0].value.size()>0) {
				return srv.response.result[0].value[0];
			}
			else {
				ROS_WARN("INTENTION_RECOGNITION - did not find values for the %s %s",f.subject.c_str(),f.predicate[0].c_str());
			}
			
		}
		else {
			ROS_WARN("INTENTION_RECOGNITION - did not find results for the %s %s",f.subject.c_str(),f.predicate[0].c_str());
		}
	}
	else {
		ROS_WARN("INTENTION_RECOGNITION - failed to contact database");
	}
	return "";
}
std::vector<std::string> ObservationsCollector::queryDatabaseVector(situation_assessment_msgs::Fact f) {
	situation_assessment_msgs::QueryDatabase srv;
	srv.request.query=f;
	if (database_service_.call(srv)) {
		if (srv.response.result.size()>0) {
			if (srv.response.result[0].value.size()>0) {
				return srv.response.result[0].value;
			}
			else {
				ROS_WARN("INTENTION_RECOGNITION - did not find values for the %s %s",f.subject.c_str(),f.predicate[0].c_str());
			}
			
		}
		else {
			ROS_WARN("INTENTION_RECOGNITION - did not find results for the %s %s",f.subject.c_str(),f.predicate[0].c_str());
		}
	}
	else {
		ROS_WARN("INTENTION_RECOGNITION - failed to contact database");
	}
	return std::vector<std::string>();
}


std::string ObservationsCollector::getAt(std::string agent) {
	situation_assessment_msgs::Fact f;
	f.model=robot_name_;
	f.subject=agent;
	f.predicate={"isAt"};

	return queryDatabase(f);
}
