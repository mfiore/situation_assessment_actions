#include <human_action_monitor/action_monitors.h>

ActionMonitors::ActionMonitors(ros::NodeHandle node_handle):node_handle_(node_handle) {
	node_handle_.getParam("/situation_assessment/robot_name",robot_name_);

	node_handle_.getParam("/situation_assessment/action_monitoring/actions_to_monitor",actions_to_monitor_);
	node_handle_.getParam("/situation_assessment/action_monitoring/trigger_distance",trigger_distance_);
	node_handle_.getParam("/situation_assessment/action_monitoring/temporal_threshold",time_threshold_);
	node_handle_.getParam("/situation_assessment/human_names",human_list_);

	ROS_INFO("HUMAN_ACTION_MONITOR robot %s",robot_name_.c_str());
	ROS_INFO("HUMAN_ACTION_MONITOR temporal temporal_threshold is %f",time_threshold_);
	
	ROS_INFO("HUMAN_ACTION_MONITOR human list:");
	for (string a:human_list_) {
		ROS_INFO("HUMAN_ACTION_MONITOR - %s",a.c_str());
			human_locations_[a+"_torso"]={"this"};
	}
	
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
		action_targets_info_[actions_to_monitor_[i]]=target;
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
 	
 	executable_actions_=msg->executable_agents_actions;
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
	srv.request.query.subject=agent+"_"+monitor_part;
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



std::vector<action_management_msgs::Action> ActionMonitors::getMoveActions() {
	std::vector<action_management_msgs::Action> move_actions;

	situation_assessment_msgs::QueryDatabase query_location;
	query_location.request.query.model=robot_name_;
	query_location.request.query.predicate.push_back("isAt");

	if (database_query_client_.call(query_location)) {
		for (int i=0;i<query_location.response.result.size();i++) {
			string entity=query_location.response.result[i].subject;
			if (human_locations_.find(entity)!=human_locations_.end()) {
				if (query_location.response.result[i].value.size()>0) {
					string new_location=query_location.response.result[i].value[0];
					if (new_location!=human_locations_.at(entity)) {
						//human has changed location
						human_locations_[entity]=new_location;
						action_management_msgs::Action action;
						action.name="move";

						common_msgs::Parameter agent_par,target_par;
						agent_par.name="main_agent";
						agent_par.value=entity;
						target_par.name="target";
						target_par.value=new_location;
						
						action.parameters={agent_par,target_par};
						move_actions.push_back(action);
					}
				}
			} 
		}
	}
	else {
		ROS_WARN("ACTION_MONITORS failed to call database");
	}
	return move_actions;
}

void ActionMonitors::actionLoop() {
	ros::Rate r(3);
	while (ros::ok()) {
		ros::spinOnce(); //Since we use spin at the start of the loop there's no need to synchronize the callbacks with a mutex
		action_management_msgs::ActionList actions_to_execute;
	
		std::vector<action_management_msgs::Action> move_actions=getMoveActions(); //we start considering move actions and then update it with other actions

		std::map<std::string,action_management_msgs::Action> agent_actions;


		for (int i=0; i<move_actions.size();i++) {
			agent_actions[move_actions[i].parameters[0].value]=move_actions[i]; //the first parameter in move is main_agent
		}

		/*if there are different actions executable on targets we will take the shortest distance 
		lower than trigger_distance_ (since we execute only one action at a time)
		*/
		std::map<std::string, double>  min_distance_targets;

		for (int i=0;i<executable_actions_.size();i++) {
			string agent=executable_actions_[i].agent;
			if (agent_actions.find(agent)==agent_actions.end()) {
				//if an agent has not already acted (ex. a move action)
				std::vector<action_management_msgs::Action> agent_executable_actions=executable_actions_[i].actions;
				for (int j=0;j<agent_executable_actions.size();j++) {
					string action_name=agent_executable_actions[j].name;
					if (action_name!="move") { //move actions are handled separately and are ignored in this loop
						std::map<std::string,std::string> parameters=getParameterMap(agent_executable_actions[j].parameters);
						string t=action_targets_info_.at(action_name);
						string action_target=parameters.at(t);

						string monitor_part=action_monitor_parts_.at(action_name);
						double distance=getDistance(agent,action_target,monitor_part);
						if (distance<trigger_distance_) {
							//distance<trigger, the action can be executed. We try to get the minimum distance
							if (min_distance_targets.find(agent)!=min_distance_targets.end()) {
								if (distance<min_distance_targets.at(agent)) {
									min_distance_targets[agent]=distance; 
									agent_actions[agent]=agent_executable_actions[j];
								}
							}
							else {
								min_distance_targets[agent]=distance;
								agent_actions[agent]=agent_executable_actions[j];
							}
						}
					}

				}
			}
		}
		for (auto aga: agent_actions) {
			if (agent_timers_.find(aga.first)==agent_timers_.end()) { //if there is no timer we create one
				agent_timers_[aga.first]=new SupervisionTimer(time_threshold_);
			}
			if (!agent_timers_.at(aga.first)->isRunning()) { //if the timer is running the agent can't execute actions 
				if (timers_threads_.find(aga.first)!=timers_threads_.end()) {
					delete timers_threads_.at(aga.first);
					timers_threads_[aga.first]=NULL;
				}
				//set postconditions
				action_management_msgs::SetPostconditions srv;
				common_msgs::ParameterList parameter_list;
				parameter_list.parameter_list=aga.second.parameters;
				srv.request.parameters=parameter_list;
				string action_name=aga.second.name;
				if (action_name=="move" || action_postconditions_services_.at(action_name).call(srv)) {
					//move has no postconditions service
					actions_to_execute.actions.push_back(aga.second);
					timers_threads_[aga.first]=new boost::thread(boost::bind(&SupervisionTimer::start,
					agent_timers_.at(aga.first)));
				}
				else {
							ROS_WARN("ACTION_MONITORS failed to set postconditions for action %s",action_name.c_str());
				}
			}
		}
		executed_actions_pub_.publish(actions_to_execute);
		r.sleep();
	}
	for (auto tt:timers_threads_) {
		agent_timers_.at(tt.first)->stop();
		tt.second->join();
		if (tt.second!=NULL) {
			delete tt.second;
		}
		delete agent_timers_.at(tt.first);
	}
}
