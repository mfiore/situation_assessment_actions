/* 
 * File:   BayesianNetwork.h
 * Author: mfiore
 *
 * Created on December 15, 2014, 9:47 AM
 */

#ifndef BAYESIANNETWORK_H
#define	BAYESIANNETWORK_H

#include <dlib/bayes_utils.h>
#include <dlib/graph_utils.h>
#include <dlib/graph.h>
#include <dlib/directed_graph.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <vector>
#include "NestedLoop.h"

using namespace dlib;
using namespace std;
using namespace bayes_node_utils;

typedef graph<dlib::set<unsigned long>::compare_1b_c, dlib::set<unsigned long>::compare_1b_c>::kernel_1a_c join_tree_type;

struct probAssignment {
    std::map<string, string> parentValues;
    string value;
    double prob;
};

class BayesianNetwork {
public:
    BayesianNetwork(int size);
    BayesianNetwork(const BayesianNetwork& orig);
    BayesianNetwork(directed_graph<bayes_node>::kernel_1a_c *bn, std::map<string, int> nodeLabels, std::map<string, std::map<string, int> > nodeValues, std::vector<bool> isTemporal);

    virtual ~BayesianNetwork();

    void loadGraph(string networkPath, std::map<string, int> nodeLabels, std::map<string, std::map<string, int> > nodeValues, std::vector<bool> isTemporal);

    void updateJoinTree();

    string getLabel(int i);
    int getIndex(string label);
    std::map<string, int> getValues(int i);
    std::map<string, int> getValues(string label);
    map<string,double> computeProbability(std::map<string, string> nodeValues);

    bool addNode(string node, std::vector<string> parents, bool isTemporal, string probabilityMode, string dominantParent);

    bool addNode(string node, std::vector<string> parents, std::vector<probAssignment> probTable);

    bool addMultiValueNode(string node, string parent, std::vector<string> values);

    setNodes(int size);


private:
    directed_graph<bayes_node>::kernel_1a_c *bn;
    join_tree_type join_tree;
    std::map<string, int> nodeLabels;
    std::map<string, std::map<string, int> > nodeValues;
    std::vector<bool> isTemporal;

    std::vector<int> numValues;

    int lastIndex;
};

#endif	/* BAYESIANNETWORK_H */

