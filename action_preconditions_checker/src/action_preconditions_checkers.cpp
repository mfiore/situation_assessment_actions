#include "action_preconditions_checker/action_preconditions_checker.h"

ActionPreconditionsChecker::ActionPreconditionsChecker(ros::NodeHandle node_handle):node_handle_(node_handle) {
	node_handle_.getParam("/situation_assessment/human_names",human_list_);
	node_handle_.getParam("/situation_assessment/action_monitoring/actions_to_monitor",actions_to_monitor_);
	node_handle_.getParam("/situation_assessment/object_names",object_list_);
	node_handle_.getParam("/situation_assessment/action_monitoring/use_database",use_database_);
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
			+actions_to_monitor_[i]+"/target",target);




		ROS_INFO("HUMAN_ACTION_MONITOR target is %s",target.c_str());
		action_targets_[actions_to_monitor_[i]]=target;

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

	std::string use_db_string=use_database_?"use the database":"use the fact topic";
	ROS_INFO("ACTION_PRECONDITIONS_CHECKER will %s to get observations",use_db_string.c_str());

	if (use_database_) {
		ROS_INFO("ACTION_PRECONDITIONS_CHECKER Connecting to database");
		database_client_=node_handle_.serviceClient<situation_assessment_msgs::QueryDatabase>("situation_assessment/query_database");
		database_client_.waitForExistence();
		ROS_INFO("ACTION_PRECONDITIONS_CHECKER Connected");
	}

	ROS_INFO("ACTION_PRECONDITIONS_CHECKER Connecting to action services");
	for (int i=0;i<actions_to_monitor_.size();i++) {
		ros::ServiceClient client_preconditions=node_handle_.serviceClient<action_management_msgs::CheckPreconditions>("/action_management/actions/"+actions_to_monitor_[i]+"/checkPreconditions");
		ROS_INFO("ACTION_PRECONDITIONS_CHECKER - waiting for action %s",actions_to_monitor_[i].c_str());
		client_preconditions.waitForExistence();
		action_preconditions_services_[actions_to_monitor_[i]]=client_preconditions;
		ROS_INFO("ACTION_PRECONDITIONS_CHECKER - connected");
	}
	ROS_INFO("ACTION_PRECONDITIONS_CHECKER connected to action services");

	if (!use_database_) {
		ROS_INFO("ACTION_PRECONDITIONS_CHECKER Subscribing to agent facts");
		fact_subscriber_=node_handle.subscribe("situation_assessment/agent_fact_list",1000,&ActionPreconditionsChecker::agentFactCallback,this);
	}

	ROS_INFO("ACTION_PRECONDITIONS_CHECKER Advertising topics");

	human_executable_actions_pub_=node_handle_.advertise<situation_assessment_actions_msgs::ExecutableActions>("/situation_assessment/human_executable_actions",1000);

	ros::Duration(5).sleep();
}

void ActionPreconditionsChecker::agentFactCallback(const situation_assessment_msgs::FactList::ConstPtr& msg) {
	std::map<std::string,bool> has_object;
	// //for each agent get if he has an object
	for (int i=0;i<human_list_.size();i++) {
		std::string human=human_list_[i];
		if (getHumanObject(human)!=""){ //this just get the object from a std::map in a thread safe way{
			has_object[human]=true;  
		}
	}

	std::map<std::string,bool> found_object;
	for (int i=0;i<msg->fact_list.size();i++) {
		situation_assessment_msgs::Fact fact=msg->fact_list[i];
		if (fact.predicate.size()>0 && fact.value.size()>0) {
			if (std::find(human_list_.begin(),human_list_.end(),fact.subject)!=human_list_.end()) {
				if (fact.predicate[0]=="has") {
					if (!has_object[fact.subject]) {
						setHumanObject(fact.subject,fact.value[0]);
						//if we have a predicate where an agent has an object, we record it in the std::map
						//in a thread safe way using setHumanObject
					}
					found_object[fact.subject]=true;
				}
			}
		}
	}
	for (int i=0;i<human_list_.size();i++) {
		if (has_object[human_list_[i]] && !found_object[human_list_[i]]) {
			setHumanObject(human_list_[i],"");
			//if an agent had an object but now he doesn't have it anymore we remove it in the std::map
		}
	}

}

void ActionPreconditionsChecker::databaseLoop() {
	// ros::Rate r(3);
	// while (ros::ok()) {
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
		// r.sleep();
	// }
}

void ActionPreconditionsChecker::start() {
	// if (use_database_) {
	// 	databaseLoop();
	// }
	// else {
		monitorLoop();
	// }
}


void ActionPreconditionsChecker::setHumanObject(std::string human,std::string object) {
	boost::lock_guard<boost::mutex> lock(mutex_human_objects_);
	human_objects_[human]=object;
}

std::string ActionPreconditionsChecker::getHumanObject(std::string human) {
	boost::lock_guard<boost::mutex> lock(mutex_human_objects_);
	return human_objects_[human];
}



void ActionPreconditionsChecker::monitorLoop() {
	ros::Rate r(1);


	while (ros::ok()) {
		if (use_database_) {
			databaseLoop();
		}

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
						if (l!=agent_location && l!="this") {
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
					target_parameter.name=action_targets_.at(action);
					target_parameter.value=object;
					parameter_list.parameter_list.push_back(target_parameter);
					parameter_list.parameter_list.push_back(agent_parameter);
					std::string human_object=getHumanObject(human);
					//if he has an object we consider it has parameter of the action (since with our mocap
					//humans can only have one hand. When they have an object they need to use it somewhere
					//or place it)
					if (human_object!="" && action_targets_.at(action)!="main_object") {
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



