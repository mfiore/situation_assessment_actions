#include "action_preconditions_checker/action_preconditions_checker.h"

ActionPreconditionsChecker::ActionPreconditionsChecker(ros::NodeHandle node_handle):node_handle_(node_handle) {
	
	//get parameters
	node_handle_.getParam("/situation_assessment/human_names",human_list_);
	node_handle_.getParam("/situation_assessment/action_monitoring/actions_to_monitor",actions_to_monitor_);
	node_handle_.getParam("/situation_assessment/object_names",object_list_);
	node_handle_.getParam("/robot_name",robot_name_);
	node_handle.getParam("/situation_assessment/locations",locations_);

	ROS_INFO("ACTION_PRECONDITIONS_CHECKER human agents:");
	for (int i=0;i<human_list_.size();i++) {
		ROS_INFO("ACTION_PRECONDITIONS_CHECKER - %s",human_list_[i].c_str());
	}
	ROS_INFO("ACTION_PRECONDITIONS_CHECKER actions to monitor:");
	for (int i=0;i<actions_to_monitor_.size();i++) {
		ROS_INFO("ACTION_PRECONDITIONS_CHECKER - %s",actions_to_monitor_[i].c_str());

		std::string target;
		node_handle_.getParam("/situation_assessment/action_monitoring/actions_details/"
			+actions_to_monitor_[i]+"/target_info",target);

		ROS_INFO("HUMAN_ACTION_MONITOR target info is %s",target.c_str());
		action_targets_info_[actions_to_monitor_[i]]=target;

	}
	ROS_INFO("ACTION_PRECONDITIONS_CHECKER object list:");
	for (int i=0;i<object_list_.size();i++) {
		ROS_INFO("ACTION_PRECONDITIONS_CHECKER - %s",object_list_[i].c_str());
	}

	for (int i=0; i<object_list_.size();i++) {
		std::vector<std::string> affordances;
		node_handle_.getParam("/situation_assessment/action_monitoring/affordances/"+object_list_[i],affordances);
		object_affordances_[object_list_[i]]=affordances;

		ROS_INFO("ACTION_PRECONDITIONS_CHECKER affordances for %s are",object_list_[i].c_str());
		for (int j=0;j<object_affordances_[object_list_[i]].size();j++) {
			ROS_INFO("ACTION_PRECONDITIONS_CHECKER - %s",object_affordances_[object_list_[i]][j].c_str());
		}
	}	

	for(std::string l : locations_) {
		std::vector<std::string> connected;
		node_handle_.getParam("/situation_assessment/connected_locations/"+l,connected);
		ROS_INFO("ACTION_PRECONDITIONS_CHECKER location %s is linked to:",l.c_str());
		for (std::string lc: connected) {
			ROS_INFO("ACTION_PRECONDITIONS_CHECKER - %s",lc.c_str());
		}
		connected_locations_[l]=connected;

	}


	ROS_INFO("ACTION_PRECONDITIONS_CHECKER Connecting to database");
	database_client_=node_handle_.serviceClient<situation_assessment_msgs::QueryDatabase>("situation_assessment/query_database");
	database_client_.waitForExistence();
	ROS_INFO("ACTION_PRECONDITIONS_CHECKER Connected");

	ROS_INFO("ACTION_PRECONDITIONS_CHECKER Connecting to action services");
	for (int i=0;i<actions_to_monitor_.size();i++) {
		ros::ServiceClient client_preconditions=node_handle_.serviceClient<action_management_msgs::CheckPreconditions>("/action_management/actions/"+actions_to_monitor_[i]+"/checkPreconditions");
		ROS_INFO("ACTION_PRECONDITIONS_CHECKER - waiting for action %s",actions_to_monitor_[i].c_str());
		client_preconditions.waitForExistence();
		action_preconditions_services_[actions_to_monitor_[i]]=client_preconditions;
		ROS_INFO("ACTION_PRECONDITIONS_CHECKER - connected");
	}
	ROS_INFO("ACTION_PRECONDITIONS_CHECKER connected to action services");

	ROS_INFO("ACTION_PRECONDITIONS_CHECKER Advertising topics");

	human_executable_actions_pub_=node_handle_.advertise<situation_assessment_actions_msgs::ExecutableActions>("/situation_assessment/human_executable_actions",1000);



	ros::Duration(5).sleep();
}



void ActionPreconditionsChecker::queryObjects() {
	situation_assessment_msgs::QueryDatabase srv;
	for (int i=0;i<human_list_.size();i++) {
		std::string human=human_list_[i];
		situation_assessment_msgs::Fact object_in_hand_fact;
		object_in_hand_fact.model=robot_name_;
		object_in_hand_fact.subject=human;
		object_in_hand_fact.predicate.push_back("has");

		situation_assessment_msgs::QueryDatabase srv;
		srv.request.query=object_in_hand_fact;
		if (!database_client_.call(srv)) {
			ROS_ERROR("ACTION_MONITORS could not contact DB");
		}
		else {
			if (srv.response.result.size()>0 && srv.response.result[0].value.size()>0) {
				setHumanObject(human,srv.response.result[0].value[0]);
			}
		}
	}
}



void ActionPreconditionsChecker::setHumanObject(std::string human,std::string object) {
	boost::lock_guard<boost::mutex> lock(mutex_human_objects_);
	human_objects_[human]=object;
}

std::string ActionPreconditionsChecker::getHumanObject(std::string human) {
	boost::lock_guard<boost::mutex> lock(mutex_human_objects_);
	return human_objects_[human];
}


bool ActionPreconditionsChecker::isConnected(std::string l1,std::string l2) {
	return std::find(connected_locations_.at(l1).begin(),
		connected_locations_.at(l1).end(),l2)!=connected_locations_.at(l1).end();
}

std::map<std::string,std::vector<action_management_msgs::Action> > ActionPreconditionsChecker::getMoveActions() {
	std::map<std::string,std::vector<action_management_msgs::Action> > agent_actions;

	for (std::string h:human_list_) {
		situation_assessment_msgs::Fact f;
		f.model=robot_name_;
		f.subject=h+"_torso";
		f.predicate={"isAt"};

		situation_assessment_msgs::QueryDatabase srv;
		srv.request.query=f;

		if (database_client_.call(srv)) {
			if (srv.response.result.size()>0 && srv.response.result[0].value.size()>0) {
				std::string agent_location=srv.response.result[0].value[0];
				for (std::string l:locations_) {
					if (l!=agent_location && l!="this" && isConnected(agent_location,l)) {
						action_management_msgs::Action a;
						a.name="move";
						std::vector<common_msgs::Parameter> parameter_list;
						common_msgs::Parameter target_parameter;
						target_parameter.name="target";
						target_parameter.value=l;
						common_msgs::Parameter agent_parameter;
						agent_parameter.name="main_agent";
						agent_parameter.value=h;
						parameter_list={agent_parameter,target_parameter};
						a.parameters=parameter_list;
						agent_actions[h].push_back(a);
					}
				}
			}
		}
		else {
			ROS_ERROR("ACTION_PRECONDITIONS_CHECKER failed to call database");
		}
	}
	return agent_actions;
}

void ActionPreconditionsChecker::monitorLoop() {
	ros::Rate r(1);

	while (ros::ok()) {
		queryObjects();
		
		std::map<std::string,std::vector<action_management_msgs::Action> > agent_actions;

		for (int i=0; i<object_list_.size();i++) {
			//For each object
			std::string object=object_list_[i];
			std::vector<std::string> affordances=object_affordances_[object];
			for (int j=0;j<affordances.size();j++) {
				//let's consider every affordance of the object (e.g. every action that can be done on it)

				std::string action=affordances[j];
				for (int k=0;k<human_list_.size();k++) {

					//for each human we will check if the preconditions of this action are satisfied

					std::string human=human_list_[k];
					common_msgs::ParameterList parameter_list;
					common_msgs::Parameter target_parameter;
					common_msgs::Parameter agent_parameter;
					//we set the parameters
					agent_parameter.name="main_agent";
					agent_parameter.value=human;
					target_parameter.name=action_targets_info_.at(action);
					target_parameter.value=object;
					parameter_list.parameter_list.push_back(target_parameter);
					parameter_list.parameter_list.push_back(agent_parameter);
					std::string human_object=getHumanObject(human);
					//if he has an object we consider it has parameter of the action (since with our mocap
					//humans can only have one hand. When they have an object they need to use it somewhere
					//or place it)
					if (human_object!="" && action_targets_info_.at(action)!="main_object") {
						//the human has an object in his hand and the target of the action is not main_object
						//this means that the object in his hand is the main object.
						//An example is place. Here the object in the human's hand is main_object and
						//the target of the action (e.g. a table) is "support_object"
						common_msgs::Parameter object_parameter;
						object_parameter.name="main_object";
						object_parameter.value=human_object;
						parameter_list.parameter_list.push_back(object_parameter);
					}
					action_management_msgs::CheckPreconditions preconditions_msg;
					preconditions_msg.request.parameters=parameter_list;
					//check the preconditions
					if (action_preconditions_services_[action].call(preconditions_msg)) {
						// ROS_INFO("ACTION_PRECONDITIONS_CHECKER called service");
						if (preconditions_msg.response.value==true) {
							// ROS_INFO("ACTION_MONITORS response is true");
							action_management_msgs::Action action_msg;
							action_msg.name=action;
							action_msg.parameters=parameter_list.parameter_list;
							agent_actions[human].push_back(action_msg);
						}	
					}
					else {
						ROS_ERROR("ACTION_PRECONDITIONS_CHECKER failed to check preconditions for action %s",action.c_str());
					}
				}
			}
		}
		//publish the actions
		situation_assessment_actions_msgs::ExecutableActions executable_actions_msg;
		for (auto aa:agent_actions) {
			situation_assessment_actions_msgs::ExecutableAgentActions agent_actions_msg;
			agent_actions_msg.agent=aa.first;
			agent_actions_msg.actions=aa.second;
			executable_actions_msg.executable_agents_actions.push_back(agent_actions_msg);
		}
		human_executable_actions_pub_.publish(executable_actions_msg);
		r.sleep();
	}

}



