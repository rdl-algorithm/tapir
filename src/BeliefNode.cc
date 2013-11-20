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

BeliefNode::BeliefNode() {
    id = currId;
    currId++;
    nParticles = 0;
    nActChildren = 0;
    distChecked = false;
    nxtActToTry = -1;
    tNNComp = -1.0;
    nnBel = NULL;
    //setEMDSig();
    tEmdSig = -1;
}

BeliefNode::BeliefNode(long id_): id(id_) {
    if (currId <= id) { currId = id; currId++; }
    distChecked = false;
    nxtActToTry = -1;
    tNNComp = -1.0;
    nnBel = NULL;
    nParticles = 0;
    //setEMDSig();
    tEmdSig = -1;
}

BeliefNode::~BeliefNode() {
    map<long, Action*>::iterator it;
    for (it = actChildren.begin(); it != actChildren.end(); it++) {
        delete it->second;
    }
    actChildren.clear();
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
    actChildren[actId] = new Action(actId, nActParticles, qVal, qValAvg);
    for (long i = 0; i < nObs; i++) {
        sstr >> tmpStr >> tmpStr >> tmpStr;
        o.clear();
        while (tmpStr != ")") {
            o.push_back(atof(tmpStr.c_str()));
            sstr >> tmpStr;
        }
        sstr >> nxtNodeId;
        BeliefNode* nxtNode = tmpNodes[nxtNodeId];
        if (nxtNode == NULL) {
            nxtNode = new BeliefNode(nxtNodeId);
            tmpNodes[nxtNodeId] = nxtNode;
        }
        actChildren[actId]->addChild(o, nxtNode);
    }
}

long BeliefNode::getUCBAct() {
    double tmpVal;
    map<long, Action*>::iterator itAct = actChildren.begin();
    double maxVal = itAct->second->avgQVal + exploreParam*sqrt(log(nParticles)/itAct->second->nParticles);
    long bestActId = itAct->first;
    itAct ++;
    for (; itAct != actChildren.end(); itAct++) {
        tmpVal = itAct->second->avgQVal + exploreParam*sqrt(log(nParticles)/itAct->second->nParticles);
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
    map<long, Action*>::iterator itAct = actChildren.begin();
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
/*
    if (nParticles < maxParticles) {
        setEMDSig();
        emdSigTmp->resize((size_t) nParticles+1);
        emdSigTmp->at<double>(nParticles, 0) = 1.0;
        //cvSet2D(emdSig, nParticles, 0, cvScalar(1.0));
        long colIdx = 1;
        vector<double>::iterator itD;
        StateVals s;
        newHistEntry->st->getVals(s);
        for (itD = s.begin(); itD != s.end(); itD++, colIdx++) {
            emdSigTmp->at<double>(nParticles, colIdx) = *itD;
            //cvSet2D(emdSig, nParticles, colIdx, cvScalar(*itD));
        }

    }
    //emdSig = emdSigTmp;
*/
    tLastAddedParticle = (double) (clock()-startTime)*10000 / CLOCKS_PER_SEC;

    particles.push_back(newHistEntry);
    nParticles ++;

}
/*
void BeliefNode::setEMDSig() {
//cerr << "nParticles: " << nParticles << endl;
    vector<HistoryEntry*>::iterator it;
    vector<double>::iterator itD;
    StateVals tmpS;
    long maxN = min(maxParticles, nParticles);
    if (tEmdSig > -1) {
        cvReleaseMat(&emdSig);
    }
    emdSig = cvCreateMat(maxN, nStVars+1, CV_32FC1);
    //emdSig->create(maxN, nStVars+1, CV_32FC1);
    //emdSigTmp = new Mat(maxN, nStVars+1, CV_32FC1);
    long rowIdx = 0; it = particles.begin();
    while (rowIdx < maxN) {
        cvSet2D(emdSig, rowIdx, 0, cvScalar(1.0));  // All particles have the same weight.
        //emdSigTmp->at<double>(rowIdx, 0) = 1.0;
        long colIdx = 1;
        (*it)->st->getVals(tmpS);
        for (itD = tmpS.begin(); itD != tmpS.end(); itD++, colIdx++) {
            cvSet2D(emdSig, rowIdx, colIdx, cvScalar(*itD));
            //emdSigTmp->at<double>(rowIdx, colIdx) = *itD;
        }
        it++; rowIdx++;
    }
    //emdSig = emdSigTmp;
    tEmdSig = (clock()-startTime)*10000/CLOCKS_PER_SEC;
}
*/

BeliefNode* BeliefNode::addChild(long actIdx, ObsVals &obs, HistoryEntry* nxtHistEntry) {
    BeliefNode* res;
    if (actChildren.find(actIdx) == actChildren.end()) {
        res = new BeliefNode();
        actChildren[actIdx] = new Action(actIdx, obs, res);
        nActChildren ++;
    }
    else {
        res = actChildren[actIdx]->getObsChild(obs);
        if (res == NULL) {
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
        actChildren[actIdx] = new Action(actIdx, obs, res);
        nActChildren ++;
    }
    else {
        res = actChildren[actIdx]->getObsChild(obs);
        if (res == NULL) {
            res = new BeliefNode();
            actChildren[actIdx]->addChild(obs, res);
        }
        actChildren[actIdx]->nParticles++;
    }
    return res;
}

HistoryEntry* BeliefNode::sampleAParticle() {
    long randIdx = GlobalResources::randIntBetween(0, nParticles - 1);
//cerr << "nodeId: " << id << " nParticles: " << nParticles << " " << particles.size() << " randIdx " << randIdx << endl;
    return particles[randIdx];
}

void BeliefNode::updateVal(long actIdx, double newVal) {
/*
map<long, Action*>::iterator itM2;
cerr << "BeliefNode::Update 2 param Before " << actChildren.size() << " ";
for (itM2 = actChildren.begin(); itM2 != actChildren.end(); itM2++) {
    cerr << "( " << itM2->first << " " << itM2->second->avgQVal << " " << itM2->second->nParticles << " ) ";
}
cerr << endl;
*/

    if (actChildren.find(actIdx) == actChildren.end()) {
//cerr << "UpdateVal No Children Act: " << actIdx << endl;
        return;
    }

    actChildren[actIdx]->updateQVal(newVal);
    map<long, Action*>::iterator itM = actChildren.begin();
    bestAvgQVal = itM->second->avgQVal;
    bestAct = itM->first;
    itM ++;
    for (; itM != actChildren.end(); itM++) {
        if (itM->second->avgQVal > bestAvgQVal) {
            bestAvgQVal = itM->second->avgQVal;
            bestAct = itM->first;
        }
    }
}

void BeliefNode::updateVal(long actIdx, double prevVal, double newVal, bool cutPart) {
/*
map<long, Action*>::iterator itM2;
cerr << "About to updateQVal bel-" << id << " act-" << actIdx << " prevVal " << prevVal << " " << newVal << " ";
cerr << "Before " << actChildren.size() << " ";
for (itM2 = actChildren.begin(); itM2 != actChildren.end(); itM2++) {
    cerr << "( " << itM2->first << " " << itM2->second->avgQVal << " " << itM2->second->nParticles << " ) ";
}
cerr << endl;
*/
//cerr << "Original bestQ " << bestAvgQVal << " " << bestAct << " After ";

    if (actChildren.find(actIdx) == actChildren.end()) {
//cerr << "UpdateValWPart NoChildren Act: " << actIdx << endl;
        return;
    }

    actChildren[actIdx]->updateQVal(prevVal, newVal, cutPart);
    map<long, Action*>::iterator itM = actChildren.begin();
    bestAvgQVal = itM->second->avgQVal;
    bestAct = itM->first;
    itM ++;
    for (; itM != actChildren.end(); itM++) {
        if (itM->second->avgQVal > bestAvgQVal) {
            bestAvgQVal = itM->second->avgQVal;
            bestAct = itM->first;
        }
    }
/*
cerr << "After ";
for (itM2 = actChildren.begin(); itM2 != actChildren.end(); itM2++) {
    cerr << "( " << itM2->first << " " << itM2->second->avgQVal << " ) ";
}
cerr << endl;
cerr << "After bestQ " << bestAvgQVal << " " << bestAct << " nPart " << nParticles << " " << particles.size() << endl;
*/
/*
vector<HistoryEntry*>::iterator itHistEntry;
for (itHistEntry = particles.begin(); itHistEntry != particles.end(); itHistEntry++) {
    (*itHistEntry)->writeln(cerr);
}
*/
}

void BeliefNode::calcBestVal() {
//cerr << "inCalcBestVal: " << actChildren.size() << "\n";
    if (actChildren.size() == 0) { return; }
    map<long, Action*>::iterator itM = actChildren.begin();
    bestAvgQVal = itM->second->avgQVal;
    bestAct = itM->first;
    itM ++;
    for (; itM != actChildren.end(); itM++) {
        if (itM->second->avgQVal > bestAvgQVal) {
            bestAvgQVal = itM->second->avgQVal;
            bestAct = itM->first;
        }
    }
}

/*
void BeliefNode::delPartNUpdateVal(HistoryEntry *histEntryToBeDeleted, double prevQVal, double newVal) {
    updateVal(histEntryToBeDeleted->actId, prevQVal, newVal, true);
    vector<HistoryEntry>::iterator itHistEntry = find(particles.begin(), particles.end(), histEntryToBeDeleted);
    if (itHistEntry != particles.end()) { particles.erase(itHistEntry); }
}
*/
BeliefNode* BeliefNode::getChild(long actIdx, ObsVals &obs) {
//cerr <<" In BeliefNode->getChild for act " << actIdx << " " << actChildren.size() << endl;
//cerr << "This Belief Node: "; writeStParticles(cerr);
    if (actChildren.find(actIdx) == actChildren.end()) {
        return NULL;
    }
    return actChildren[actIdx]->getObsChild(obs);
}

void BeliefNode::getChildren(queue<BeliefNode*> &res) {
    map<long, Action*>::iterator itAct;
    for (itAct = actChildren.begin(); itAct != actChildren.end(); itAct++) {
        itAct->second->getChildren(res);
    }
}

double BeliefNode::distL1Independent(BeliefNode *b) {
    double dist = 0.0;
    vector<HistoryEntry*>::iterator itPart1, itPart2;
    for (itPart1 = b->particles.begin(); itPart1 != b->particles.end(); itPart1++) {
        for (itPart2 = particles.begin(); itPart2 != particles.end(); itPart2++) {
            dist = dist + (*itPart1)->st->distL1((*itPart2)->st);
        }
        dist = dist/(nParticles*b->nParticles);
    }
    return dist;
}
/*
void BeliefNode::delParticle(HistoryEntry *histEntry, long actId, double qVal) {
    if(actChildren[actId]->nParticles > 1) {
        actChildren[actId]->delParticle(qVal);
    }
    else {
        invalidActChildren[actId] = actChildren[actId];
        actChildren.erase(actId);
        nActChildren --;
    }
    nParticles --;
}
*/

long BeliefNode::getNxtActToTry() {
    nxtActToTry ++;
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
    map<long, Action*>::iterator itAct;
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
