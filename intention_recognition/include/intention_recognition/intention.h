#ifndef INTENTION_H
#define INTENTION_H

#include <string>
#include <vector>
using namespace std;
class Intention {

public:
	Intention(string name, vector<string> contexts>):contexts_(contexts),name_(name) {

	}

	vector<string> contexts_;
	string name_;
}


#endif