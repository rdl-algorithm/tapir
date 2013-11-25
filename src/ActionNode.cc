#include "ActionNode.h"
#include "BeliefNode.h"

using namespace std;

ActionNode::ActionNode(long actId, ObsVals &obs, BeliefNode* nxtBelNode) :
        ActionNode(actId, 1, 0.0, 0.0) {
    obsChildren.push_back(new ObservationEdge(obs, nxtBelNode));
}

ActionNode::ActionNode(long actId, long nParticles, double qVal, double avgQVal) :
        actId(actId), nParticles(nParticles), qVal(qVal), avgQVal(avgQVal),
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

void ActionNode::delParticle(double delVal) {
    qVal = qVal - delVal;
    nParticles--;
    avgQVal = qVal / (double) nParticles;
}

bool ActionNode::isAct(long aIdx) {
    if (aIdx == actId) {
        return true;
    } else {
        return false;
    }
}

void ActionNode::addChild(ObsVals &obs, BeliefNode* nxtBelNode) {
    obsChildren.push_back(new ObservationEdge(obs, nxtBelNode));
}

BeliefNode* ActionNode::getObsChild(ObsVals &obs) {
    vector<ObservationEdge*>::iterator itObs;
    for (itObs = obsChildren.begin(); itObs != obsChildren.end(); itObs++) {
        if ((*itObs)->isObs(obs)) {
            return (*itObs)->getNodeChild();
        }
    }
    return nullptr;
}

void ActionNode::getChildren(queue<BeliefNode*> &res) {
    vector<ObservationEdge*>::iterator it;
    for (it = obsChildren.begin(); it != obsChildren.end(); it++) {
        (*it)->getChildren(res);
    }
}

void ActionNode::write(ostream &os) {
    os << "A " << actId << " " << nParticles << " " << qVal << " " << avgQVal
            << " " << obsChildren.size() << " ";
}

void ActionNode::writeNGetChildren(ostream &os, queue<BeliefNode*> &res) {
    write(os);
    vector<ObservationEdge*>::iterator it;
    for (it = obsChildren.begin(); it != obsChildren.end(); it++) {
        (*it)->writeNGetChildren(os, res);
    }
    os << endl;
}
