#include "BeliefTree.hpp"

#include <fstream>
using std::ifstream;
#include <ostream>
using std::ostream;
using std::endl;
#include <queue>
using std::queue;
#include <string>
using std::getline;
using std::string;
#include <sstream>
using std::stringstream;
#include <vector>
using std::vector;

#include "BeliefNode.hpp"
class Histories;

BeliefTree::BeliefTree() :
            root(new BeliefNode()),
            allNodes() {
    allNodes.push_back(root);
}

BeliefTree::~BeliefTree() {
    queue<BeliefNode*> tmpNodes;
    tmpNodes.push(root);
    deleteHelper(tmpNodes);
}

void BeliefTree::deleteHelper(queue<BeliefNode*> &tmpNodes) {
    while (!tmpNodes.empty()) {
        BeliefNode *aNode = tmpNodes.front();
        tmpNodes.pop();
        aNode->getChildren(tmpNodes);
        delete aNode;
    }
}

void BeliefTree::readPolicy(ifstream &inFile, Histories *allHist) {
    string tmpStr, usrStr;
    getline(inFile, tmpStr);
    while (tmpStr.find("BELIEFTREE-BEGIN") == string::npos) {
        getline(inFile, tmpStr);
    }
    getline(inFile, tmpStr);

    double nNodes;
    stringstream sstr(tmpStr);
    sstr >> usrStr >> nNodes;
    sstr.flush();
    sstr.clear();
    vector<BeliefNode*> allNodes(nNodes, nullptr);
    allNodes[0] = root;

    // Set root node
    long nodeId;
    getline(inFile, tmpStr);
    sstr.str(tmpStr);
    sstr >> usrStr >> nodeId;
    root->set(sstr, allHist);
    for (long i = 0; i < root->nActChildren; i++) {
        getline(inFile, tmpStr);
        root->setAct(tmpStr, allNodes);
    }
    root->calcBestVal();

    // Set the rest of the node
    getline(inFile, tmpStr);
    while (tmpStr.find("BELIEFTREE-END") == string::npos) {
        sstr.str(tmpStr);
        sstr >> usrStr >> nodeId;
        //cerr << "node: " << nodeId << endl;
        allNodes[nodeId]->set(sstr, allHist);
        for (long i = 0; i < allNodes[nodeId]->nActChildren; i++) {
            getline(inFile, tmpStr);
            allNodes[nodeId]->setAct(tmpStr, allNodes);
        }
        allNodes[nodeId]->calcBestVal();
        getline(inFile, tmpStr);
    }
}

void BeliefTree::write(ostream &os) {
    os << "#nodes: " << BeliefNode::currId << endl;
    queue<BeliefNode*> toBeWritten;
    toBeWritten.push(root);
    writeHelp(toBeWritten, os);
}

void BeliefTree::writeHelp(queue<BeliefNode*> &tmpNodes, ostream &os) {
    while (!tmpNodes.empty()) {
        BeliefNode *aNode = tmpNodes.front();
        tmpNodes.pop();
        aNode->writeNGetChildren(os, tmpNodes);
    }
}
