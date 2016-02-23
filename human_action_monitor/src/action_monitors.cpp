#include <human_action_monitor/action_monitors.h>

ActionMonitors::ActionMonitors(ros::NodeHandle node_handle):node_handle_(node_handle) {
	node_handle_.getParam("/situation_assessment/human_agents",human_list_);
	node_handle_.getParam("/situation_assessment/actions_to_monitor",actions_to_monitor_);
	node_handle_.getParam("/situation_assessment/object_list",object_list_);
	node_handle_.getParam("/situation_assessment/trigger_distance",trigger_distance_);
	node_handle_.getParam("/situation_assessment/human_action_monitor/use_database",use_database_);
	node_handle_.getParam("/robot_name",robot_name_);

	ROS_INFO("HUMAN_ACTION_MONITOR human agents:");
	for (int i=0;i<human_list_.size();i++) {
		ROS_INFO("HUMAN_ACTION_MONITOR - %s",human_list_[i].c_str());
	}
	ROS_INFO("HUMAN_ACTION_MONITOR actions to monitor:");
	for (int i=0;i<actions_to_monitor_.size();i++) {
		ROS_INFO("HUMAN_ACTION_MONITOR - %s",actions_to_monitor_[i].c_str());
	}
	ROS_INFO("HUMAN_ACTION_MONITOR object list:");
	for (int i=0;i<object_list_.size();i++) {
		ROS_INFO("HUMAN_ACTION_MONITOR - %s",object_list_[i].c_str());
	}

	for (int i=0; i<object_list_.size();i++) {
		vector<string> affordances;
		node_handle_.getParam("/situation_assessment/affordances/"+object_list_[i],affordances);
		object_affordances_[object_list_[i]]=affordances;

		ROS_INFO("HUMAN_ACTION_MONITOR affordances for %s are",object_list_[i].c_str());
		for (int j=0;j<object_affordances_[object_list_[i]].size();j++) {
			ROS_INFO("HUMAN_ACTION_MONITOR - %s",object_affordances_[object_list_[i]][j].c_str());
		}
	}	

	ROS_INFO("HUMAN_ACTION_MONITOR Connecting to database");
	database_client_=node_handle_.serviceClient<situation_assessment_msgs::QueryDatabase>("situation_assessment/query_database");
	database_client_.waitForExistence();
	ROS_INFO("HUMAN_ACTION_MONITOR Connected");

	ROS_INFO("HUMAN_ACTION_MONITOR Connecting to action services");
	for (int i=0;i<actions_to_monitor_.size();i++) {
		ros::ServiceClient client_preconditions=node_handle_.serviceClient<action_management_msgs::CheckPreconditions>("/supervision/actions/"+actions_to_monitor_[i]+"/check_preconditions");
		ros::ServiceClient client_postconditions=node_handle_.serviceClient<action_management_msgs::SetPostconditions>("/supervision/actions/"+actions_to_monitor_[i]+"/set_postconditions");
		action_preconditions_services_[actions_to_monitor_[i]]=client_preconditions;
		action_postconditions_services_[actions_to_monitor_[i]]=client_postconditions;
	}
	ROS_INFO("HUMAN_ACTION_MONITOR connected to action services");




	ROS_INFO("HUMAN_ACTION_MONITOR Subscribing to agent facts");
	fact_subscriber_=node_handle.subscribe("situation_assessment/agent_fact_list",1000,&ActionMonitors::agentFactCallback,this);

	ROS_INFO("HUMAN_ACTION_MONITOR Advertising topics");
	for (int i=0; i<human_list_.size();i++) {
		ros::Publisher pub=node_handle_.advertise<action_management_msgs::Action>("/situation_assessment/"+human_list_[i]+"/action_performed",1000);
		human_action_topics_[human_list_[i]]=pub;
	}

}

void ActionMonitors::start() {
	if (use_database_) {
		databaseLoop();
	}
	else {
		monitorLoop();
	}
}
void ActionMonitors::agentFactCallback(const situation_assessment_msgs::FactList::ConstPtr& msg) {
	map<string,bool> has_object;
	for (int i=0;i<human_list_.size();i++) {
		string human=human_list_[i];
		if (getHumanObject(human)!="") {
			has_object[human]=true;
		}
	}

	map<string,bool> found_object;

	for (int i=0;i<msg->fact_list.size();i++) {
		situation_assessment_msgs::Fact fact=msg->fact_list[i];
		if (fact.predicate.size()>2 && fact.value.size()>0) {
			if (fact.predicate[0]=="distance" && fact.predicate[1]=="hand") {
				if (std::find(human_list_.begin(),human_list_.end(),fact.subject)!=human_list_.end()) {
					if (std::find(object_list_.begin(),object_list_.end(),fact.predicate[2])!=object_list_.end()) {
						setObjectDistance(fact.subject,fact.predicate[2],boost::lexical_cast<double>(fact.value[0]));
					}
				}
			}
		}
		else if (fact.predicate.size()>0 && fact.value.size()>0) {
			if (std::find(human_list_.begin(),human_list_.end(),fact.subject)!=human_list_.end()) {
				if (fact.predicate[0]=="has") {
					if (!has_object[fact.subject]) {
						setHumanObject(fact.subject,fact.value[0]);
					}
					found_object[fact.subject]=true;
				}
			}
		}
	}
	for (int i=0;i<human_list_.size();i++) {
		if (has_object[human_list_[i]] && !found_object[human_list_[i]]) {
			setHumanObject(human_list_[i],"");
		}
	}

}

void ActionMonitors::databaseLoop() {
	ros::Rate r(3);
	while (ros::ok()) {
		situation_assessment_msgs::QueryDatabase srv;
		for (int i=0;i<human_list_.size();i++) {
			string human=human_list_[i];
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

			for (int i=0; i<object_list_.size();i++) {
				string object=object_list_[i];
				situation_assessment_msgs::Fact distance_to_object_fact;
				distance_to_object_fact.model=robot_name_;
				distance_to_object_fact.subject="human";
				distance_to_object_fact.predicate.push_back("distance");
				distance_to_object_fact.predicate.push_back("hand");
				distance_to_object_fact.predicate.push_back(object);	

				srv.request.query=distance_to_object_fact;
				if (!database_client_.call(srv)) {
					ROS_ERROR("ACTION_MONITORS could not contact DB");
				}
				else {
					if (srv.response.result.size()>0 && srv.response.result[0].value.size()>0) {
						setObjectDistance(human,object,boost::lexical_cast<double>(srv.response.result[0].value[0]));
					}
				}
			}
		}
		r.sleep();
	}
}

void ActionMonitors::setHumanObject(string human,string object) {
	boost::lock_guard<boost::mutex> lock(mutex_human_objects_);
	human_objects_[human]=object;
}

string ActionMonitors::getHumanObject(string human) {
	boost::lock_guard<boost::mutex> lock(mutex_human_objects_);
	return human_objects_[human];
}

void ActionMonitors::setObjectDistance(string human, string object, double distance) {
	boost::lock_guard<boost::mutex> lock(mutex_object_distance_);
	std::pair<string,string> p(human,object);
	object_distances_[p]=distance;
}

double ActionMonitors::getObjectDistance(string human,string object) {
	boost::lock_guard<boost::mutex> lock(mutex_object_distance_);
	std::pair<string,string> p(human,object);
	return object_distances_[p];
}



void ActionMonitors::monitorLoop() {
	ros::Rate r(3);

	while (ros::ok()) {
		for (int i=0; i<object_list_.size();i++) {
			string object=object_list_[i];
			vector<string> affordances=object_affordances_[object];
			for (int j=0;j<affordances.size();j++) {
				string action=affordances[j];
				for (int k=0;k<human_list_.size();k++) {
					string human=human_list_[k];
					common_msgs::ParameterList parameter_list;
					common_msgs::Parameter target_parameter;
					target_parameter.name="target";
					target_parameter.value=object;
					parameter_list.parameter_list.push_back(target_parameter);
					string human_object=getHumanObject(human);
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
							double hand_distance=getObjectDistance(human,object);
							if (hand_distance<trigger_distance_) {
								action_management_msgs::SetPostconditions postconditions_msg;
								postconditions_msg.request.parameters=parameter_list;
								if (action_postconditions_services_[action].call(postconditions_msg)) {
									action_management_msgs::Action action_msg;
									action_msg.name=action;
									action_msg.parameters=parameter_list.parameter_list;
									human_action_topics_[human].publish(action_msg);
									ROS_INFO("ACTION_MONITORS agent %s completed action %s %s %s",human.c_str(),action.c_str(),human_object.c_str(),object.c_str());
								}
								else {
									ROS_ERROR("ACTION_MONITORS failed to set postconditions for action %s",action.c_str());
								}
							}
						}	
					}
					else {
						ROS_ERROR("ACTION_MONITORS failed to check preconditions for action %s",action.c_str());
					}
				}
			}
		}
		r.sleep();
	}

}



