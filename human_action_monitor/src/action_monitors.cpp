#include <human_action_monitor/action_monitors.h>

ActionMonitors::ActionMonitors(ros::NodeHandle node_handle):node_handle_(node_handle) {
	node_handle_.getParam("/situation_assessment/robot_name",robot_name_);

	node_handle_.getParam("/situation_assessment/action_monitoring/actions_to_monitor",actions_to_monitor_);
	node_handle_.getParam("/situation_assessment/action_monitoring/trigger_distance",trigger_distance_);
	node_handle_.getParam("/situation_assessment/action_monitoring/use_database",use_database_);

	ROS_INFO("HUMAN_ACTION_MONITOR robot %s",robot_name_.c_str());
	ROS_INFO("HUMAN_ACTION_MONITOR actions to monitor:");
	for (int i=0;i<actions_to_monitor_.size();i++) {
		ROS_INFO("HUMAN_ACTION_MONITOR - %s",actions_to_monitor_[i].c_str());

		std::string target;
		std::string monitor_part;

		node_handle_.getParam("/situation_assessment/action_monitoring/actions_details/"
			+actions_to_monitor_[i]+"/target",target);
		node_handle_.getParam("/situation_assessment/action_monitoring/actions_details/"
			+actions_to_monitor_[i]+"/monitor_part",monitor_part);

		ROS_INFO("HUMAN_ACTION_MONITOR target is %s",target.c_str());
		ROS_INFO("HUMAN_ACTION_MONITOR monitor part is %s",monitor_part.c_str());
		action_targets_[actions_to_monitor_[i]]=target;
		action_monitor_parts_[actions_to_monitor_[i]]=monitor_part;

	}

	ROS_INFO("HUMAN_ACTION_MONITOR Connecting to database");
	database_query_client_=node_handle_.serviceClient<situation_assessment_msgs::QueryDatabase>("situation_assessment/query_database");
	database_query_client_.waitForExistence();
	ROS_INFO("HUMAN_ACTION_MONITOR Connected");

	ROS_INFO("HUMAN_ACTION_MONITOR Connecting to action services");
	for (int i=0;i<actions_to_monitor_.size();i++) {
		ros::ServiceClient client_postconditions=node_handle_.serviceClient<action_management_msgs::SetPostconditions>("/action_management/actions/"+actions_to_monitor_[i]+"/setPostconditions");
		client_postconditions.waitForExistence();
		action_postconditions_services_[actions_to_monitor_[i]]=client_postconditions;
	}
	ROS_INFO("HUMAN_ACTION_MONITOR connected to action services");

	executable_actions_subscriber_=node_handle_.subscribe("situation_assessment/human_executable_actions", 
		1000,&ActionMonitors::executableActionsCallback,this);

	executed_actions_pub_=node_handle_.advertise<action_management_msgs::ActionList>("/situation_assessment/humans_executed_actions",1000);
}

void ActionMonitors::executableActionsCallback(
 	const situation_assessment_actions_msgs::ExecutableActions::ConstPtr& msg) {
 	
 	executable_actions_=msg->executable_actions;
}

std::map<std::string,std::string> ActionMonitors::getParameterMap(
	std::vector<common_msgs::Parameter> parameter_message) {
	std::map<std::string,std::string> parameters;

	for (int i = 0; i < parameter_message.size(); ++i)
	{
		parameters[parameter_message[i].name]=parameter_message[i].value;
	}
	return parameters;
}

double ActionMonitors::getDistance(string agent, string target, string monitor_part) {
	situation_assessment_msgs::QueryDatabase srv;
	srv.request.query.model=robot_name_;
	srv.request.query.subject=agent;
	srv.request.query.predicate.push_back("distance");
	// srv.request.query.predicate.push_back(monitor_part);
	srv.request.query.predicate.push_back(target);

	// ROS_INFO("MODEL %s subject %s predicate %s %s",robot_name_.c_str(),agent.c_str(),srv.request.query.predicate[0].c_str(),
		// srv.request.query.predicate[1].c_str());
	if (database_query_client_.call(srv)) {
		if (srv.response.result.size()>0) {
			if (srv.response.result[0].value.size()>0) {
				return std::stod(srv.response.result[0].value[0]);
			}
			else {
				ROS_WARN("ACTION_MONITORS no values in database response");
			}		
		}
		else {
			ROS_WARN("ACTION_MONITORS no results in database response");			
		}
	}
	else {
		ROS_WARN("ACTION_MONITORS failed to contact database");
	}
	return 10000;

}
void ActionMonitors::actionLoop() {
	ros::Rate r(3);
	while (ros::ok()) {
		ros::spinOnce();
		action_management_msgs::ActionList actions_to_execute;
		for (int i=0; i<executable_actions_.size();i++) {
			string action_name=executable_actions_[i].name;
			std::map<std::string,std::string> parameters=getParameterMap(executable_actions_[i].parameters);
			string t=action_targets_.at(action_name);
			string action_target=parameters.at(t);
			string agent=parameters.at("main_agent");

			string monitor_part=action_monitor_parts_.at(action_name);
			double distance=getDistance(agent,action_target,monitor_part);
			if (distance<trigger_distance_) {
				action_management_msgs::SetPostconditions srv;
				common_msgs::ParameterList parameter_list;
				parameter_list.parameter_list=executable_actions_[i].parameters;
				srv.request.parameters=parameter_list;
				if (action_postconditions_services_.at(action_name).call(srv)) {
					actions_to_execute.actions.push_back(executable_actions_[i]);
				}
				else {
					ROS_WARN("ACTION_MONITORS failed to set postconditions for action %s",action_name.c_str());
				}
			}
		}
		executed_actions_pub_.publish(actions_to_execute);
		r.sleep();
	}
}