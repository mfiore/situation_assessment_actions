#ifndef INTENTION_RECOGNITION_H
#define INTENTION_RECOGNITION_H

#include <string>
#include <vector>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

using namespace std;
class IntentionRecognition {

public:
	IntentionRecognition(string human, vector<string> context_list, vector<string> intention_lis_, vector<string> action_list, 
		vector<Intention> intentions_, ros::NodeHandle node_handle_);

	void start();
	void stop();
private:
	void createNetwork();
	void createContextNodes();
	void createIntentionNodes();
	void createActionNodes();
	void createObservationNodes();

	void checkForNewActions();

	bool shouldRun();

	double getQValue(string intention, string action);

	ros::NodeHandle node_handle_;

	boost::mutex mutex_should_run_;
	bool should_run_;

	BayesianNetwork bayes_;
	string human_;
	vector<string> context_list_;
	vector<Intention> intentions_;
	vector<string> action_list_;
	vector<string> intention_list_;

	vector<string> distance_values_;
	vector<string> intention_values_;

	map<string,ros::ServiceClient> mdp_action_value_services_;


};

#endif