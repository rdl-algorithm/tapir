#include <cmath>
#include <cstdlib>

#include "BeliefNode.h"
#include "GlobalResources.h"

using namespace std;

long BeliefNode::currId = 0;
double BeliefNode::exploreParam = 1.0;
clock_t BeliefNode::startTime = clock();
long BeliefNode::maxParticles = 0;
long BeliefNode::nStVars = 0;

BeliefNode::BeliefNode() :
        BeliefNode(currId) {
}

BeliefNode::BeliefNode(long id) :
        distChecked(false), id(id), nParticles(0), nActChildren(0),
        nxtActToTry(-1), bestAvgQVal(0), bestAct(-1), tLastAddedParticle(0),
        tNNComp(-1.0), tEmdSig(-1.0), nnBel(nullptr), particles(), actChildren() {
    if (currId <= id) {
        currId = id + 1;
    }
}

BeliefNode::~BeliefNode() {
    map<long, ActionNode*>::iterator it;
    for (it = actChildren.begin(); it != actChildren.end(); it++) {
        delete it->second;
    }
}

void BeliefNode::set(stringstream &sstr, Histories *allHist) {
    string tmpStr;
    long seqId, entryId;
    sstr >> nParticles >> nActChildren >> tmpStr;
    for (long i = 0; i < nParticles; i++) {
        sstr >> tmpStr >> seqId >> entryId >> tmpStr;
        HistoryEntry *tmpHistEntry = allHist->getHistoryEntry(seqId, entryId);
        tmpHistEntry->setBelNode(this);
        particles.push_back(tmpHistEntry);
    }
}

void BeliefNode::setAct(string str, vector<BeliefNode*> &tmpNodes) {
    stringstream sstr(str);
    string tmpStr;
    ObsVals o;
    long actId, nActParticles, nObs, nxtNodeId;
    double qVal, qValAvg;
    sstr >> tmpStr >> actId >> nActParticles >> qVal >> qValAvg >> nObs;
    actChildren[actId] = new ActionNode(actId, nActParticles, qVal, qValAvg);
    for (long i = 0; i < nObs; i++) {
        sstr >> tmpStr >> tmpStr >> tmpStr;
        o.clear();
        while (tmpStr != ")") {
            o.push_back(atof(tmpStr.c_str()));
            sstr >> tmpStr;
        }
        sstr >> nxtNodeId;
        BeliefNode* nxtNode = tmpNodes[nxtNodeId];
        if (nxtNode == nullptr) {
            nxtNode = new BeliefNode(nxtNodeId);
            tmpNodes[nxtNodeId] = nxtNode;
        }
        actChildren[actId]->addChild(o, nxtNode);
    }
}

long BeliefNode::getUCBAct() {
    double tmpVal;
    map<long, ActionNode*>::iterator itAct = actChildren.begin();
    double maxVal = itAct->second->avgQVal
            + exploreParam * sqrt(log(nParticles) / itAct->second->nParticles);
    long bestActId = itAct->first;
    itAct++;
    for (; itAct != actChildren.end(); itAct++) {
        tmpVal = itAct->second->avgQVal
                + exploreParam
                        * sqrt(log(nParticles) / itAct->second->nParticles);
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
    map<long, ActionNode*>::iterator itAct = actChildren.begin();
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
    tLastAddedParticle =
            (double) (clock() - startTime) * 10000 / CLOCKS_PER_SEC;

    particles.push_back(newHistEntry);
    nParticles++;

}

BeliefNode* BeliefNode::addChild(long actIdx, ObsVals &obs,
        HistoryEntry* nxtHistEntry) {
    BeliefNode* res;
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

BeliefNode* BeliefNode::addChild(long actIdx, ObsVals &obs) {
    BeliefNode* res;
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

HistoryEntry* BeliefNode::sampleAParticle() {
    return particles[GlobalResources::randIntBetween(0, nParticles - 1)];
}

void BeliefNode::updateVal(long actIdx, double newVal) {
    if (actChildren.find(actIdx) == actChildren.end()) {
        //cerr << "UpdateVal No Children Act: " << actIdx << endl;
        return;
    }
    actChildren[actIdx]->updateQVal(newVal);
    map<long, ActionNode*>::iterator itM = actChildren.begin();
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
    map<long, ActionNode*>::iterator itM = actChildren.begin();
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

BeliefNode* BeliefNode::getChild(long actIdx, ObsVals &obs) {
    //cerr <<" In BeliefNode->getChild for act " << actIdx << " " << actChildren.size() << endl;
    //cerr << "This Belief Node: "; writeStParticles(cerr);
    if (actChildren.find(actIdx) == actChildren.end()) {
        return nullptr;
    }
    return actChildren[actIdx]->getObsChild(obs);
}

void BeliefNode::getChildren(queue<BeliefNode*> &res) {
    map<long, ActionNode*>::iterator itAct;
    for (itAct = actChildren.begin(); itAct != actChildren.end(); itAct++) {
        itAct->second->getChildren(res);
    }
}

double BeliefNode::distL1Independent(BeliefNode *b) {
    double dist = 0.0;
    vector<HistoryEntry*>::iterator itPart1, itPart2;
    for (itPart1 = b->particles.begin(); itPart1 != b->particles.end();
            itPart1++) {
        for (itPart2 = particles.begin(); itPart2 != particles.end();
                itPart2++) {
            dist = dist + (*itPart1)->st->distL1((*itPart2)->st);
        }
        dist = dist / (nParticles * b->nParticles);
    }
    return dist;
}

long BeliefNode::getNxtActToTry() {
    nxtActToTry++;
    return nxtActToTry;
}

void BeliefNode::write(ostream &os) {
    os << "Node " << id << " " << nParticles << " " << nActChildren << " : ";
    vector<HistoryEntry*>::iterator it;
    for (it = particles.begin(); it != particles.end(); it++) {
        os << "( " << (*it)->getSeqId() << " " << (*it)->getId() << " ) ";
    }
    os << endl;
}

void BeliefNode::writeNGetChildren(ostream &os, queue<BeliefNode*> &res) {
    write(os);
    map<long, ActionNode*>::iterator itAct;
    for (itAct = actChildren.begin(); itAct != actChildren.end(); itAct++) {
        itAct->second->writeNGetChildren(os, res);
    }
}

void BeliefNode::writeStParticles(ostream &os) {
    vector<HistoryEntry*>::iterator it;
    for (it = particles.begin(); it != particles.end(); it++) {
        (*it)->writeSt(os);
    }
    os << endl;
}
