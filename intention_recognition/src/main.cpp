/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: mfiore
 *
 * Created on April 27, 2016, 1:59 PM
 */

#include <iostream>


#include <cstdlib>

#include "intention_recognition/BayesianNetwork.h"
#include "pomdp_lib/Mdp.h"


using namespace std;

/*
 * 
 */

vector<string> intention_list_;
vector<string> context_list_;
vector<Intention> intentions_;
vector<string> human_list_;
vector<string> action_list_;
vector<string> target_list_;
map<string,vector<string> > target_action_list_;
string robot_name_;

void createIntentions(ros::NodeHandle *node_handle) {
	node_handle->getParams("/situation_assessment/intention_recognition/intention_list",intention_list_);
	node_handle->getParams("/situation_assessment/intention_recognition/context_list",context_list_);

	for (string intention: intention_list_) {
		vector<string> linked_contexts
		node_handle->getParams("/situation_assessment/intention_recognition/intention_list/"+intention+"/linked_contexts",linked_contexts);
		Intention i(name,linked_contexts);
		intentions_.push_back(i);
	}
}

void createActions(ros::NodeHandle* node_handle) {
	node_handle_.getParams("/situation_assessment/intention_recognition/action_list",action_list_);
	node_handle_.getParam("/situation_assessment/intention_recognition/target_list",target_list_);

	for (string target:action_list_) {
		vector<string> target_actions;
		node_handle_.getParam("/situation_assessment/target_actions/"+target,target_actions);
		target_action_list_[target]=target_actions;
	}

}
int main(int argc, char** argv) {
	ros::init(argc,argv,"intention_recognition");
	ros::NodeHandle node_handle;
	createIntentions(node_handle);
	node_handle_.getParam("/situation_assessment/human_agents",human_list_);
	node_handle_.getParam("/robot_name",robot_name_);


	IntentionRecognition intention_recognition;


    return 0;
}