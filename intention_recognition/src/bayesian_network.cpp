/* 
 * File:   BayesianNetwork.cpp
 * Author: mfiore
 * 
 * Created on December 15, 2014, 9:47 AM
 */

#include "BayesianNetwork.h"
#include <iostream>

BayesianNetwork::BayesianNetwork(int size) {
    lastIndex = -1;
    bn = new directed_graph<bayes_node>::kernel_1a_c();
    bn->set_number_of_nodes(size);

}

BayesianNetwork::setNodes(int size) {
    bn->set_number_of_nodes(size);
}

BayesianNetwork::BayesianNetwork(const BayesianNetwork& orig) {
}

BayesianNetwork::~BayesianNetwork() {
}

void BayesianNetwork::loadGraph(string networkPath, std::map<string, int> nodeLabels, std::map<string, std::map<string, int> > nodeValues, std::vector<bool> isTemporal) {
    try {

        this->nodeLabels = nodeLabels;
        this->nodeValues = nodeValues;
        this->isTemporal = isTemporal;

        ifstream fin(networkPath.c_str(), ios::binary);

        // Note that the saved networks produced by the bayes_net_gui_ex.cpp example can be deserialized
        // into a network.  So you can make your networks using that GUI if you like.
        cout << "Loading the network from disk..." << endl;
        deserialize(*bn, fin);

        cout << "Number of nodes in the network: " << bn->number_of_nodes() << endl;




    } catch (exception& e) {
        cout << "exception thrown: " << e.what() << endl;

    }
}

void BayesianNetwork::updateJoinTree() {
    create_moral_graph(*bn, join_tree);
    create_join_tree(join_tree, join_tree);
}

map<string,double> BayesianNetwork::computeProbability(std::map<string, string> evidence) {
    // Now to make things more interesting let's say that we have discovered that the C 
    // node really has a value of 1.  That is to say, we now have evidence that 
    // C is 1.  We can represent this in the network using the following two function
    // calls.
    for (int i = 0; i < bn->number_of_nodes(); i++) {
        set_node_as_nonevidence(*bn, i);
        //                cout << "Number of parents of node " << i << "= " << bn->node(i).number_of_parents() << "\n";
        //                cout << "CPT stil filled? " << node_cpt_filled_out(*bn, i) << "\n";
    }
    for (std::map<string, string>::iterator i = evidence.begin(); i != evidence.end(); i++) {
        set_node_value(*bn, nodeLabels[i->first], nodeValues[i->first][i->second]);
        set_node_as_evidence(*bn, nodeLabels[i->first]);
    }




    // Now we have a proper join_tree we can use it to obtain a solution to our
    // bayesian network.  Doing this is as simple as declaring an instance of
    // the bayesian_network_join_tree object as follows:
    bayesian_network_join_tree solution(*bn, join_tree);

    // now print out the probabilities for each node
    cout << "Using the join tree algorithm with solution size:" << solution.number_of_nodes() << "\n";



    map<string,double> result;

    for (std::map<string, std::map<string, int> >::iterator i = nodeValues.begin(); i != nodeValues.end(); i++) {
        // print out the probability distribution for node i.  
        int nodeIndex = nodeLabels[i->first];

        for (int j = 0; j < i->second.size(); j++) {
            cout << "p(node " << i->first << "==" << j << ") = " << solution.probability(nodeIndex)(j) << " \n";
            result[i->first]=solution.probability(nodeIndex)(j);
        }

    }
    return result;
}

string BayesianNetwork::getLabel(int i) {
    for (std::map<string, int>::iterator node = nodeLabels.begin(); node != nodeLabels.end(); node++) {
        if (node->second == i) {
            return node->first;
        }
    }
    return "";
}

int BayesianNetwork::getIndex(string label) {
    return nodeLabels[label];
}

std::map<string, int> BayesianNetwork::getValues(int i) {
    for (std::map<string, int>::iterator node = nodeLabels.begin(); node != nodeLabels.end(); node++) {
        if (node->second == i) {

            return nodeValues[node->first];
        }
    }
    return std::map<string, int>();

}

std::map<string, int> BayesianNetwork::getValues(string label) {
    return nodeValues[label];
}

BayesianNetwork::BayesianNetwork(directed_graph<bayes_node>::kernel_1a_c *bn, std::map<string, int> nodeLabels, std::map<string, std::map<string, int> > nodeValues, std::vector<bool> isTemporal) {
    this->nodeLabels = nodeLabels;
    this->nodeValues = nodeValues;
    this->isTemporal = isTemporal;
    this->bn = bn;

}

bool BayesianNetwork::addMultiValueNode(string node, string parent, std::vector<string> values) {
    //create new node
    lastIndex = lastIndex + 1;
    int nodeIndex = lastIndex;

    nodeLabels[node] = nodeIndex;
    set_node_num_values(*bn, nodeIndex, values.size());
    numValues.push_back(values.size());

    std::map<string, int> actualValues;
    for (int i = 0; i < values.size(); i++) {
        actualValues[values[i]] = i;
    }
    nodeValues[node] = actualValues;

    bn->add_edge(nodeLabels[parent], nodeIndex);

    std::map<string, int> parentValues = nodeValues[parent];

    int numParts = 0;
    for (int i = 1; i < values.size() + 1; i++) {
        numParts = numParts + i;
    }
    double inc = 1.0 / numParts;

    assignment parent_state;
    parent_state.add(nodeLabels[parent], 1);
    for (int j = 0; j < values.size(); j++) {
        double p = inc * (j + 1);
        set_node_probability(*bn, nodeIndex, j, parent_state, p);

    }

    parent_state.clear();
    parent_state.add(nodeLabels[parent], 0);
    for (int j = 0; j < values.size(); j++) {
        double p = inc * (values.size() - j);
        set_node_probability(*bn, nodeIndex, j, parent_state, p);

    }
    cout << "CPT for node " << node << " " << nodeIndex << "= " << node_cpt_filled_out(*bn, nodeIndex) << parent << "\n";
    cout << "p(" << nodeIndex << ")=\n";
    conditional_probability_table * table = (&bn->node(nodeIndex).data.table());

    for (int i = 0; i < parentValues.size(); i++) {
        assignment parent_state;
        parent_state.add(nodeLabels[parent], i);
        for (int j = 0; j < values.size(); j++) {
            cout << table->probability(j, parent_state) << "\n";

        }
    }
}

//creates a new node with different parents and a choice of  of different probability tables. For now we deal with only binary nodes.
//More complex nodes need to be created by hand

bool BayesianNetwork::addNode(string node, std::vector<string> parents, bool isTemporal, string probabilityMode, string dominantParent) {
    //check if parents exist  and load their values in tableVariables;
    std::vector<std::vector<int> > tableVariables;

    int dominantParentIndex;

    for (int i = 0; i < parents.size(); i++) {
        if (parents[i] == dominantParent) {
            dominantParentIndex = i;
            break;
        }
    }

    BOOST_FOREACH(string parent, parents) {
        if (nodeLabels.find(parent) == nodeLabels.end()) return false;
        std::vector<int> values;
        for (int i = 0; i < nodeValues[parent].size(); i++) {
            values.push_back(i);
        }
        tableVariables.push_back(values);
    }
    //get the matrix corresponding to the parent's combination table
    std::vector<std::vector<int> > tableAssignments;
    if (tableVariables.size() > 0) {
        NestedLoop<int> loop(tableVariables);
        tableAssignments = loop.buildMatrix();
    }
    //create new node
    lastIndex = lastIndex + 1;
    int nodeIndex = lastIndex;

    nodeLabels[node] = nodeIndex;
    set_node_num_values(*bn, nodeIndex, 2);
    numValues.push_back(2);

    std::map<string, int> actualValues;
    actualValues["f"] = 0;
    actualValues["t"] = 1;
    nodeValues[node] = actualValues;



    //add parent edges

    BOOST_FOREACH(string parent, parents) {
        bn->add_edge(nodeLabels[parent], nodeIndex);
    }
    this->isTemporal.push_back(isTemporal);


    if (probabilityMode == "uniform") {
        double p = 1.0 / 2;
        if (parents.size() == 0) {
            assignment parent_state;
            set_node_probability(*bn, nodeIndex, 1, parent_state, p);
            set_node_probability(*bn, nodeIndex, 0, parent_state, p);
        } else {
            for (int i = 0; i < tableAssignments.size(); i++) {
                assignment parent_state;
                for (int j = 0; j < tableAssignments[i].size() - 1; j++) {
                    parent_state.add(nodeLabels[parents[j]], tableAssignments[i][j]);
                }
                set_node_probability(*bn, nodeIndex, 1, parent_state, p);
                set_node_probability(*bn, nodeIndex, 0, parent_state, p);

            }
        }
    } else if (probabilityMode == "equalWeight") {
        if (parents.size() == 0) {
            double p = 0.7;
            assignment parent_state;
            set_node_probability(*bn, nodeIndex, 1, parent_state, p);
            set_node_probability(*bn, nodeIndex, 0, parent_state, 1 - p);
        } else {
            for (int i = 0; i < tableAssignments.size(); i++) {
                assignment parent_state;
                int npos = 0;
                for (int j = 0; j < tableAssignments[i].size(); j++) {
                    parent_state.add(nodeLabels[parents[j]], tableAssignments[i][j]);
                    if (tableAssignments[i][j] == 1) {
                        npos++;
                    }
                }
                double p;
                if (npos == 0) {
                    p = 0.1;
                } else if (npos == parents.size()) {
                    p = 0.9;
                } else {
                    p = npos / parents.size();
                }
                set_node_probability(*bn, nodeIndex, 1, parent_state, p);
                set_node_probability(*bn, nodeIndex, 0, parent_state, 1 - p);
            }
        }
    } else if (probabilityMode == "or") {
        if (parents.size() == 0) {
            double p = 0.7;
            assignment parent_state;
            set_node_probability(*bn, nodeIndex, 1, parent_state, p);
            set_node_probability(*bn, nodeIndex, 0, parent_state, 1 - p);
        } else {
            for (int i = 0; i < tableAssignments.size(); i++) {
                assignment parent_state;
                int npos = 0;
                for (int j = 0; j < tableAssignments[i].size(); j++) {
                    parent_state.add(nodeLabels[parents[j]], tableAssignments[i][j]);
                    if (tableAssignments[i][j] == 1) {
                        npos++;
                    }
                }
                double p;
                if (npos == 0) {
                    p = 0.3;
                } else if (npos == parents.size()) {
                    p = 0.7;
                } else {
                    p = 0.5 + (0.2 * npos / parents.size());
                }
                set_node_probability(*bn, nodeIndex, 1, parent_state, p);
                set_node_probability(*bn, nodeIndex, 0, parent_state, 1 - p);
            }
        }
    } else if (probabilityMode == "dominantParent") {
        for (int i = 0; i < tableAssignments.size(); i++) {
            assignment parent_state;
            bool dominantParentTrue = false;

            int npos = 0;
            for (int j = 0; j < tableAssignments[i].size(); j++) {
                parent_state.add(nodeLabels[parents[j]], tableAssignments[i][j]);
                if (tableAssignments[i][j] == 1) {
                    if (dominantParentIndex == j) {
                        dominantParentTrue = true;
                    }
                    npos++;
                }
            }
            double p;
            if (dominantParentTrue) {
                p = 0.8;
            } else {
                p = 0.2;
            }

            set_node_probability(*bn, nodeIndex, 1, parent_state, p);
            set_node_probability(*bn, nodeIndex, 0, parent_state, 1 - p);
        }
    }
    cout << "CPT for node " << node << " " << nodeIndex << " = " << node_cpt_filled_out(*bn, nodeIndex) << "\n";
    for (int i = 0; i < parents.size(); i++) {
        cout << parents[i] << " ";
    }
    if (parents.size()>0)   cout << "\n";
    conditional_probability_table * table = (&bn->node(nodeIndex).data.table());

    if (parents.size() > 0) {
        for (int i = 0; i < tableAssignments.size(); i++) {
            assignment parent_state;
            for (int j = 0; j < tableAssignments[i].size(); j++) {
                parent_state.add(nodeLabels[parents[j]], tableAssignments[i][j]);
            }
            cout << table->probability(0, parent_state) << "\n";
            cout << table->probability(1, parent_state) << "\n";
        }
    }
    else {
        assignment parent_state;
        cout<<table->probability(0,parent_state)<<"\n";
        cout<<table->probability(1,parent_state)<<"\n";
    }
}

bool BayesianNetwork::addNode(string node, std::vector<string> parents, std::vector<probAssignment> probTable) {
    //create new node
    lastIndex = lastIndex + 1;
    int nodeIndex = lastIndex;

    nodeLabels[node] = nodeIndex;
    set_node_num_values(*bn, nodeIndex, 2);
    numValues.push_back(2);

    std::map<string, int> actualValues;
    actualValues["f"] = 0;
    actualValues["t"] = 1;
    nodeValues[node] = actualValues;

    //add parent edges
    for (string parent : parents) {
        bn->add_edge(nodeLabels[parent], nodeIndex);
    }

    cout << "CPT for node " << node << " " << nodeIndex << "\n";
    for (int i = 0; i < parents.size(); i++) {
        cout << parents[i] << " ";
    }
    cout << "\n";

    for (probAssignment row : probTable) {
        assignment parentState;
        for (auto parentValue : row.parentValues) {
            int parentIndex = nodeLabels[parentValue.first];
            int value = nodeValues[parentValue.first][parentValue.second];
            parentState.add(parentIndex, value);
        }
        set_node_probability(*bn, nodeIndex, 1, parentState, row.prob);
        set_node_probability(*bn, nodeIndex, 0, parentState, 1 - row.prob);
        cout << row.prob << "\n";
    }
    //    for (auto a:nodeValues[node]) {
    //        cout<<a.first<<" "<<a.second<<"\n";
    //    }
}
