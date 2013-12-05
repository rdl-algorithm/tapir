#include "BeliefNode.hpp"

#include <cmath>                        // for log, sqrt
#include <ctime>                        // for clock, CLOCKS_PER_SEC, clock_t

#include <iostream>                     // for cerr, endl, operator<<, basic_ostream, ostream
#include <map>                          // for map, _Rb_tree_iterator, map<>::iterator
#include <queue>                        // for queue
#include <utility>                      // for pair
#include <vector>                       // for vector, vector<>::iterator

#include "ActionNode.hpp"               // for ActionNode
#include "GlobalResources.hpp"          // for GlobalResources
#include "HistoryEntry.hpp"             // for HistoryEntry
#include "Observation.hpp"              // for Observation
#include "State.hpp"                    // for State
#include "StateWrapper.hpp"             // for StateWrapper

using std::cerr;
using std::endl;

long BeliefNode::currId = 0;
double BeliefNode::exploreParam = 1.0;
std::clock_t BeliefNode::startTime = std::clock();
long BeliefNode::maxParticles = 0;
long BeliefNode::nStVars = 0;

BeliefNode::BeliefNode() :
    BeliefNode(currId) {
}

BeliefNode::BeliefNode(long id) :
    distChecked(false),
    id(id),
    nParticles(0),
    nActChildren(0),
    nxtActToTry(-1),
    bestAvgQVal(0),
    bestAct(-1),
    tLastAddedParticle(0),
    tNNComp(-1.0),
    tEmdSig(-1.0),
    nnBel(nullptr),
    particles(),
    actChildren() {
    if (currId <= id) {
        currId = id + 1;
    }
}

BeliefNode::~BeliefNode() {
    std::map<long, ActionNode *>::iterator it;
    for (it = actChildren.begin(); it != actChildren.end(); it++) {
        delete it->second;
    }
}

long BeliefNode::getUCBAct() {
    double tmpVal;
    std::map<long, ActionNode *>::iterator itAct = actChildren.begin();
    double maxVal = itAct->second->avgQVal
                    + exploreParam
                    * std::sqrt(
                        std::log(nParticles) / itAct->second->nParticles);
    long bestActId = itAct->first;
    itAct++;
    for (; itAct != actChildren.end(); itAct++) {
        tmpVal = itAct->second->avgQVal
                 + exploreParam
                 * std::sqrt(
                     std::log(nParticles)
                     / itAct->second->nParticles);
        if (maxVal < tmpVal) {
            maxVal = tmpVal;
            bestActId = itAct->first;
        }
    }
    return bestActId;
}

long BeliefNode::getBestAct() {
    if (actChildren.size() == 0) {
        cerr << "No children - could not retrieve best action." << endl;
        return -1;
    }
    std::map<long, ActionNode *>::iterator itAct = actChildren.begin();
    double bestQVal = itAct->second->avgQVal;
    long bestActId = itAct->first;
    itAct++;
    for (; itAct != actChildren.end(); itAct++) {
        if (bestQVal < itAct->second->avgQVal) {
            bestQVal = itAct->second->avgQVal;
            bestActId = itAct->first;
        }
    }
    return bestActId;
}

void BeliefNode::add(HistoryEntry *newHistEntry) {
    tLastAddedParticle = (double) (std::clock() - startTime)
                         * 10000/ CLOCKS_PER_SEC;

    particles.push_back(newHistEntry);
    nParticles++;

}

BeliefNode *BeliefNode::addChild(long actIdx, Observation &obs,
                                 HistoryEntry *nxtHistEntry) {
    BeliefNode *res;
    if (actChildren.find(actIdx) == actChildren.end()) {
        res = new BeliefNode();
        actChildren[actIdx] = new ActionNode(actIdx, obs, res);
        nActChildren++;
    } else {
        res = actChildren[actIdx]->getObsChild(obs);
        if (res == nullptr) {
            res = new BeliefNode();
            actChildren[actIdx]->addChild(obs, res);
        }
        actChildren[actIdx]->nParticles++;
    }
    res->add(nxtHistEntry);
    return res;
}

BeliefNode *BeliefNode::addChild(long actIdx, Observation &obs) {
    BeliefNode *res;
    if (actChildren.find(actIdx) == actChildren.end()) {
        res = new BeliefNode();
        actChildren[actIdx] = new ActionNode(actIdx, obs, res);
        nActChildren++;
    } else {
        res = actChildren[actIdx]->getObsChild(obs);
        if (res == nullptr) {
            res = new BeliefNode();
            actChildren[actIdx]->addChild(obs, res);
        }
        actChildren[actIdx]->nParticles++;
    }
    return res;
}

HistoryEntry *BeliefNode::sampleAParticle() {
    return particles[GlobalResources::randIntBetween(0, nParticles - 1)];
}

void BeliefNode::updateVal(long actIdx, double newVal) {
    if (actChildren.find(actIdx) == actChildren.end()) {
        //cerr << "UpdateVal No Children Act: " << actIdx << endl;
        return;
    }
    actChildren[actIdx]->updateQVal(newVal);
    calcBestVal();
}

void BeliefNode::updateVal(long actIdx, double prevVal, double newVal,
                           bool cutPart) {
    if (actChildren.find(actIdx) == actChildren.end()) {
        //cerr << "UpdateValWPart NoChildren Act: " << actIdx << endl;
        return;
    }
    actChildren[actIdx]->updateQVal(prevVal, newVal, cutPart);
    calcBestVal();
}

void BeliefNode::calcBestVal() {
    //cerr << "inCalcBestVal: " << actChildren.size() << "\n";
    if (actChildren.size() == 0) {
        return;
    }
    std::map<long, ActionNode *>::iterator itM = actChildren.begin();
    bestAvgQVal = itM->second->avgQVal;
    bestAct = itM->first;
    itM++;
    for (; itM != actChildren.end(); itM++) {
        if (itM->second->avgQVal > bestAvgQVal) {
            bestAvgQVal = itM->second->avgQVal;
            bestAct = itM->first;
        }
    }
}

BeliefNode *BeliefNode::getChild(long actIdx, Observation &obs) {
    //cerr <<" In BeliefNode->getChild for act " << actIdx << " " << actChildren.size() << endl;
    //cerr << "This Belief Node: "; writeStParticles(cerr);
    if (actChildren.find(actIdx) == actChildren.end()) {
        return nullptr;
    }
    return actChildren[actIdx]->getObsChild(obs);
}

void BeliefNode::enqueueChildren(std::queue<BeliefNode *> &res) {
    std::map<long, ActionNode *>::iterator itAct;
    for (itAct = actChildren.begin(); itAct != actChildren.end(); itAct++) {
        itAct->second->enqueueChildren(res);
    }
}

double BeliefNode::distL1Independent(BeliefNode *b) {
    double distance = 0.0;
    std::vector<HistoryEntry *>::iterator itPart1, itPart2;
    for (itPart1 = b->particles.begin(); itPart1 != b->particles.end();
            itPart1++) {
        for (itPart2 = particles.begin(); itPart2 != particles.end();
                itPart2++) {
            distance += (*itPart1)->st->distL1((*itPart2)->st);
        }
    }
    return distance / (nParticles * b->nParticles);
}

State BeliefNode::particleMean() {
    State meanState(nStVars);
    for (HistoryEntry *entry : particles) {
        State s;
        entry->st->getVals(s);
        for (int i = 0; i < nStVars; i++) {
            meanState[i] += s[i];
        }
    }
    for (int i = 0; i < nStVars; i++) {
        meanState[i] /= nParticles;
    }
    return meanState;
}

long BeliefNode::getNxtActToTry() {
    nxtActToTry++;
    return nxtActToTry;
}
