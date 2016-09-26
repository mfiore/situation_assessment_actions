/*
author: mfiore
This node is used to give an estimation of the most likelyl intentions and actions of an agent.
External nodes can request inference by using the service:

situation_assessment/monitor_intentions  of type situation_assessment_msgs::StartMonitorIntentions
if the agent in this message is new, the node will create a new IG, if not it will update the previous IG of the
agent with the new information

situation_assessment/stop_monitor_intentions stops the inference process of an agent

it's important to specify in the situation_assessment_actions.yaml msg the link between each intention and the mdps (if mdps are used)
if there is no mdp the system can't infer action and should receive only actions in the message

when using MDPS variables must be defined as varname_predicate in order to work with the DB.
For example, if there is a variable called agent_isAt the system will contact the Database looking for a fact
like (agent isAt *)
*/

#include <ros/ros.h>
#include "intention_recognition/intention_recognition.h"
#include <vector>
#include <string>
#include <utility>

//steps
//- recognizing intentions with action stream
//- recognizing actions
//- correcting divergent belief
//- adding parameters to MDPs

typedef std::vector<std::string> StringVector;
typedef std::vector<double> DoubleVector;
typedef std::pair<std::string,std::string>  StringPair;



int main(int argc, char ** argv) {
	ros::init(argc,argv,"intention_recognition");

	ROS_INFO("INTENTION_RECOGNITION - started node");

	ros::NodeHandle n;
	IntentionRecognition intention_recognition(n);
	intention_recognition.intentionLoop();

	return 0;
}