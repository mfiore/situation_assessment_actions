/* Manages the IntentionGraph of an agent*/

#ifndef INTENTION_RECOGNITION_H
#define INTENTION_RECOGNITION

#include <ros/ros.h>
#include "intention_recognition/observations_collector.h"
#include "situation_assessment_actions_msgs/StartMonitorIntentions.h"
#include "situation_assessment_actions_msgs/ExecutableActions.h"
#include "situation_assessment_actions_msgs/ExecutableAgentActions.h"
#include  "situation_assessment_actions_msgs/IntentionGraphResult.h"
#include  "situation_assessment_actions_msgs/IntentionProb.h"
#include "IntentionGraph.h"
#include "Mdp.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp> 
#include <boost/thread/lock_guard.hpp> 
#include <set>
#include <vector>
#include <string>
#include <map>
#include <std_msgs/Float32.h>


typedef std::vector<std::string> StringVector;
typedef std::pair<std::string,std::string> StringPair;

class IntentionRecognition {
public:
	IntentionRecognition(ros::NodeHandle node_handle);
	void intentionLoop();

private:

	//loads all the known MDPs, by reading their model files
	void loadMdp();

	/*when the executable actions of an agent change, its IG need to be recreated to account for the new
	actions*/
	void actionCallback(const situation_assessment_actions_msgs::ExecutableActions::ConstPtr& msg);

	//creates an IG for an agent
	void createIntentionGraph(std::string agent,std::vector<std::string> actions);

	//turns an action msg into a string, which is used by the IG and the MDPS
	//ex: an action to pick a mug will become something like agent_pick_mug
	std::string createActionString(action_management_msgs::Action a);

	//gets all the intention graphs
	std::map<std::string,IntentionGraph*> getIntentionGraphs();

	ros::NodeHandle node_handle_;

	//gets observations for the IG (context, state, observation nodes)
	ObservationsCollector observations_collector_;

	StringVector contexts_; //list of contexts

	//links an intention to a vector of context that influence it
	std::map<std::string, std::vector<std::string> > linked_contexts_;

	//link an intention and a context to the influence of that context on 
	std::map<StringPair,double> intention_conditional_probabilities_;

	//links an intention to an MDP
	std::map<std::string,Mdp*> mdp_map_;

	StringVector intention_list_;	//list of intentions
	//links an agent to its IG
	std::map<std::string,IntentionGraph*> agents_intentions_;

	//Subscriber for executable action
	ros::Subscriber action_sub_;

	//we have a publisher that publishes all the intentions in a topic.
	//then we have a different topic for each intention, in order to plot them 
	ros::Publisher pub_intentions_; 
	std::map<std::string, ros::Publisher*> pub_intentions_plot_;

	//synchronizes IGs
	boost::mutex mutex_igs_;

	//links an agent to the actions he can execute
	std::map<std::string,std::vector<action_management_msgs::Action> > executable_actions_;
	std::vector<std::string> locations_; //list of locations
	std::vector<std::string> humans_; //list of humans
	std::map<std::string,std::string> human_locations_; //links a human to his location
};
#endif