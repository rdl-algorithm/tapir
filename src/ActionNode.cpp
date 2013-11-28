#include "ActionNode.hpp"

#include <queue>
using std::queue;
#include <vector>
using std::vector;

#include "Observation.hpp"
#include "ObservationEdge.hpp"
class BeliefNode;

ActionNode::ActionNode() :
            ActionNode(-1, 0, 0.0, 0.0) {
}

ActionNode::ActionNode(long actId, Observation &obs, BeliefNode* nxtBelNode) :
            ActionNode(actId, 1, 0.0, 0.0) {
    obsChildren.push_back(new ObservationEdge(obs, nxtBelNode));
}

ActionNode::ActionNode(long actId, long nParticles, double qVal, double avgQVal) :
            actId(actId),
            nParticles(nParticles),
            qVal(qVal),
            avgQVal(avgQVal),
            obsChildren() {
}

ActionNode::~ActionNode() {
    vector<ObservationEdge*>::iterator it;
    for (it = obsChildren.begin(); it != obsChildren.end(); it++) {
        delete (*it);
    }
    obsChildren.clear();
}

void ActionNode::updateQVal(double newVal) {
    qVal = qVal + newVal;
    if (nParticles > 0) {
        avgQVal = qVal / (double) nParticles;
    } else {
        avgQVal = 0;
    }
}

void ActionNode::updateQVal(double prevVal, double newVal,
        bool reduceParticles) {
    if (reduceParticles) {
        nParticles--;
    }
    //cerr << "nParticles: " << nParticles << " fr " << qVal << " " << avgQVal;
    qVal = qVal - prevVal + newVal;
    if (nParticles > 0) {
        avgQVal = qVal / (double) nParticles;
    } else {
        avgQVal = 0;
    }
    //cerr << " become " << qVal << " " << avgQVal << endl;
}

bool ActionNode::isAct(long aIdx) {
    if (aIdx == actId) {
        return true;
    } else {
        return false;
    }
}

void ActionNode::addChild(ObservationEdge *edge) {
    obsChildren.push_back(edge);
}

void ActionNode::addChild(Observation &obs, BeliefNode* nxtBelNode) {
    obsChildren.push_back(new ObservationEdge(obs, nxtBelNode));
}

BeliefNode* ActionNode::getObsChild(Observation &obs) {
    vector<ObservationEdge*>::iterator itObs;
    for (itObs = obsChildren.begin(); itObs != obsChildren.end(); itObs++) {
        if ((*itObs)->isObs(obs)) {
            return (*itObs)->getNodeChild();
        }
    }
    return nullptr;
}

void ActionNode::enqueueChildren(queue<BeliefNode*> &res) {
    vector<ObservationEdge*>::iterator it;
    for (it = obsChildren.begin(); it != obsChildren.end(); it++) {
        (*it)->enqueueChildren(res);
    }
}
