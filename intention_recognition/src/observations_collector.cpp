#include "intention_recognition/observations_collector.h"

ObservationsCollector::ObservationsCollector(ros::NodeHandle node_handle):node_handle_(node_handle) {
	database_service_=node_handle_.serviceClient<situation_assessment_msgs::QueryDatabase>("situation_assessment/query_database");
	ROS_INFO("INTENTION_RECOGNITION - waiting for database");
	database_service_.waitForExistence();
	ROS_INFO("INTENTION_RECOGNITION - connected");

	node_handle.getParam("/robot_name",robot_name_);

	std::vector<std::string> locations;
	node_handle.getParam("/situation_assessment/locations",locations);
	ROS_INFO("INTENTION_RECOGNITION - Location list");
	for (std::string l:locations){
		int depth;
		node_handle.getParam("/situation_assessment/locations_depth/"+l,depth);
		depth_areas_[l]=depth;
		ROS_INFO("INTENTION_RECOGNITION - location %s depth %d",l.c_str(),depth);
	}



}

//gets the current evidence for an IG
std::map<std::string,std::string> ObservationsCollector::getEvidence(std::string agent, IntentionGraph* ig) {
	std::vector<std::string> observation_nodes=ig->getObservationNodes();
	std::map<std::string,std::string> evidence;

		for (std::string node : observation_nodes) {
		std::vector<std::string> node_parts=StringOperations::stringSplit(node,'_');
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

		evidence[node]=queryDatabase(f);
	}
	std::vector<std::string> context_nodes=ig->getContextNodes();
	for (std::string node: context_nodes) {
		situation_assessment_msgs::Fact f;
		f.subject=agent;
		f.model=robot_name_;
		f.predicate.push_back(node);

		evidence[node]=queryDatabase(f);
	}
	return evidence;
}

//gets the initial state of an MDP
std::map<std::string,std::string> ObservationsCollector::getInitialState(std::string agent, std::vector<Mdp*> mdps) {
	std::map<std::string,std::string> initial_state;

	situation_assessment_msgs::Fact has_fact;
	has_fact.model=robot_name_;
	has_fact.subject=agent;
	has_fact.predicate={"has"};

	string human_object=queryDatabase(has_fact);

	for (Mdp* m: mdps) {
		std::vector<std::string> original_vars=m->getOriginalVars();
		for (std::string v : original_vars) {
			if (initial_state.find(v)==initial_state.end()) {

				std::vector<std::string> v_parts=StringOperations::stringSplit(v,'_');
				situation_assessment_msgs::Fact f;
				f.model=robot_name_;
				f.subject=v_parts[0];
				if (v_parts[1]=="isAt") {
					if (v_parts[0]==human_object) {
						//if the human has an object its location is "human" in our formalism
						initial_state[v]=agent;
					}
					else {
						//at is isInArea in the system at the moment. Since an agent can be in more than one area
						//at the same time (as areas can be nested) we will take just the maximum depth area
						//(e.g. the table is in the living room. Both are areas. Living room has depth 1 and table
						//has depth 2 so we take the table as value for the variable)
						f.predicate.push_back("isInArea");
						std::vector<string> areas=queryDatabaseVector(f);
						int max_depth=0;
						string max_depth_area;

						for (string a:areas) {
							int depth=depth_areas_.at(a);
							if (depth>max_depth) {
								max_depth=depth;
								max_depth_area=a;
							}
	 					}
	 					initial_state[v]=max_depth_area;
				  	}
				}
				else {
					for (int i=1; i<v_parts.size();i++){
						f.predicate.push_back(v_parts[i]);
					}
					initial_state[v]=queryDatabase(f);
				}
				// ROS_INFO("INTENTION_RECOGNITION subject %s",f.subject.c_str());
				// ROS_INFO("INTENTION_RECOGNITION predicate:");
				for (string p:f.predicate) {
					// ROS_INFO("INTENTION_RECOGNITION %s",p.c_str());					
				}
				// ROS_INFO("INTENTION_RECOGNITION value:");
				for (string p:f.value) {
					// ROS_INFO("INTENTION_RECOGNITION %s",p.c_str());					
				}
			}
		}
	}
	return initial_state;
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
