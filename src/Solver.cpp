#include "Solver.hpp"

#include <cfloat>
#include <climits>
#include <cmath>
using std::exp;
using std::pow;
#include <cstdlib>
using std::exit;
#include <ctime>
using std::clock;
using std::clock_t;

#include <algorithm>
using std::max;
#include <fstream>
using std::ifstream;
#include <iostream>
using std::cerr;
using std::endl;
using std::ostream;
#include <map>
using std::map;
#include <set>
using std::set;
#include <vector>
using std::vector;

#include "BeliefNode.hpp"
#include "BeliefTree.hpp"
#include "GlobalResources.hpp"
#include "Histories.hpp"
#include "HistoryEntry.hpp"
#include "HistorySequence.hpp"
#include "Model.hpp"
#include "StatePool.hpp"
#include "StateWrapper.hpp"
#include "TextSerializer.hpp"

Solver::Solver(Model *model) :
            model(model),
            policy(new BeliefTree()),
            allHistories(new Histories()),
            allStates(new StatePool()),
            rolloutUsed(ROLLOUT_RANDHEURISTIC),
            exploreCoef(model->getExploreCoef()),
            cRollout { 1.0, 1.0 },
            wRollout { 1.0, 1.0 },
            pRollout { 0.5, 0.5 },
            nUsedRollout { 1, 1 } {
}

Solver::Solver(Model *model, const char *polFile) :
            Solver(model) {
    ifstream inFile;
    inFile.open(polFile);
    if (!inFile.is_open()) {
        cerr << "Fail to open " << polFile << "\n";
        exit(1);
    }
    TextSerializer serializer(allStates);
    serializer.load(*allStates, inFile);
    serializer.load(*allHistories, inFile);
    policy->readPolicy(inFile, allHistories);
    inFile.close();
}

Solver::~Solver() {
    delete allStates;
    delete allHistories;
    delete policy;
}

void Solver::genPol(long maxTrials, double depthTh) {
    double disc = model->getDiscount();

    // Set root to have at least one particle for each children.
    BeliefNode *root = policy->getRoot();
    BeliefNode *currNode;
    HistoryEntry *rootHistEntry;
    HistoryEntry *currHistEntry;
    HistorySequence *currHistSeq;

    Observation obs;
    State sVals, nxtSVals;
    StateWrapper *st;
    for (unsigned long i = 0; i < model->getNActions(); i++) {
        model->sampleAnInitState(sVals);
        st = allStates->add(sVals);
        rootHistEntry = new HistoryEntry(st);
        rootHistEntry->partOfBelNode = root;
        root->add(rootHistEntry);
        st->addInfo(rootHistEntry);
        st->addInfo(root);
        currHistSeq = new HistorySequence(rootHistEntry, 0);
        allHistories->add(currHistSeq);
        double immediateRew;
        //cerr << "sample-" << i << " " << sVals[0] << " " << sVals[1] << endl;
        bool isTerm = model->getNextState(sVals, i, &immediateRew, nxtSVals,
                obs);
        rootHistEntry->disc = 1.0;
        rootHistEntry->rew = immediateRew;

        st = allStates->add(nxtSVals);
        currHistEntry = currHistSeq->addEntry(i, obs, st);
        currNode = root->addChild(i, obs, currHistEntry);
        //currNode->setEMDSig(model->nParticles, model->nStVars);
        policy->allNodes.push_back(currNode);
        currHistEntry->partOfBelNode = currNode;
        currHistEntry->disc = disc;
        currHistEntry->rew = 0.0;
        st->addInfo(currHistEntry);
        st->addInfo(currNode);

        if (isTerm) {
            currHistEntry->rew = model->getReward(nxtSVals);
            currHistEntry->qVal = disc * currHistEntry->rew;
        } else {
            currHistEntry->rew = model->getReward(nxtSVals);
            currHistEntry->qVal = disc * currHistEntry->rew;
        }

        rootHistEntry->qVal = immediateRew + currHistEntry->qVal;
        root->updateVal(i, rootHistEntry->qVal);
    }

    // Start expanding the tree.
    for (long i = 0; i < maxTrials; i++) {
        singleSearch(disc, depthTh);
    }
}

void Solver::singleSearch(double discount, double depthTh) {
    HistorySequence *currHistSeq;
    long actIdx;
    double immediateRew;
    Observation obs;
    State nxtSVals;
//cerr << "STARTSINGLESEARCH\n";
    BeliefNode *currNode = policy->getRoot();
    BeliefNode *nxtNode;
    double initStartVal = currNode->bestAvgQVal;
    model->sampleAnInitState(nxtSVals);
//cerr << "sampledInit: " << nxtSVals[0] << " " << nxtSVals[1] << endl;
    StateWrapper *nxtSt = allStates->add(nxtSVals);
//cerr << "st: " << nxtSt->s[0] << " " << nxtSt->s[1] << endl;
    HistoryEntry *currHistEntry = new HistoryEntry(nxtSt);
//cerr << "currHistEntry: "  << currHistEntry->st->s[0] << " " << currHistEntry->st->s[1] << endl;
    currHistEntry->partOfBelNode = currNode;
    currNode->add(currHistEntry);
    nxtSt->addInfo(currHistEntry);
    nxtSt->addInfo(currNode);

    double currDiscFactor = 1.0;
    currHistEntry->disc = currDiscFactor;
    currHistSeq = new HistorySequence(currHistEntry, 0);
    allHistories->add(currHistSeq);

    bool rolloutUsed = false;
    bool done = false;
    while (!done && currDiscFactor > depthTh) {
//cerr << "#children: " << model->nActions << " " << currNode->actChildren.size() << endl;
        if (model->getNActions() == currNode->actChildren.size()) {
//cerr << "Start UCB act\n";
            actIdx = currNode->getUCBAct();
            nxtSVals.vals.clear();
            obs.clear();
            done = model->getNextState(currHistEntry->st->state, actIdx,
                    &immediateRew, nxtSVals, obs);
            currHistEntry->rew = immediateRew;
            nxtSt = allStates->add(nxtSVals);
            currHistEntry = currHistSeq->addEntry(actIdx, obs, nxtSt);

            currDiscFactor = currDiscFactor * discount;
            currHistEntry->disc = currDiscFactor;
            nxtNode = currNode->addChild(actIdx, obs, currHistEntry);
            //nxtNode->setEMDSig(model->nParticles, model->nStVars);
            policy->allNodes.push_back(nxtNode);
            currHistEntry->partOfBelNode = nxtNode;
//nxtNode->add(currHistEntry);
            nxtSt->addInfo(currHistEntry);
            nxtSt->addInfo(nxtNode);
            currNode = nxtNode;
            if (done) {
                currHistEntry->rew = model->getReward(nxtSVals);
                currHistEntry->qVal = currHistEntry->disc * currHistEntry->rew;
            }
        } else {
//cerr << "Start rolloutAct\n";
            double qVal;
            nxtSVals.vals.clear();
            obs.clear();
//cerr << "About to rollout from " << currHistEntry->st->s[0] << " " << currHistEntry->st->s[1] << endl;
            actIdx = getRolloutAct(currNode, currHistEntry->st->state,
                    currDiscFactor, discount, nxtSVals, obs, &immediateRew,
                    &qVal);
            currHistEntry->rew = immediateRew;
//cerr << "nxtSVals size: " << nxtSVals.size() << endl;
            nxtSt = allStates->add(nxtSVals);
            currHistEntry = currHistSeq->addEntry(actIdx, obs, nxtSt);
            nxtNode = currNode->addChild(actIdx, obs, currHistEntry);
            //nxtNode->setEMDSig(model->nParticles, model->nStVars);
            policy->allNodes.push_back(nxtNode);
            currHistEntry->partOfBelNode = nxtNode;
//nxtNode->add(currHistEntry);
            currHistEntry->disc = currDiscFactor * discount;
            currHistEntry->qVal = qVal;
            nxtSt->addInfo(currHistEntry);
            nxtSt->addInfo(nxtNode);
            rolloutUsed = true;
            done = true;
        }
    }

    backup(currHistSeq);
    if (rolloutUsed) {
        updWeightRolloutAct(policy->root->bestAvgQVal - initStartVal);
    }
    rolloutUsed = false;
}

long Solver::getRolloutAct(BeliefNode *belNode, State &s, double startDisc,
        double disc, State &nxtSVals, Observation &obs, double *immediateRew,
        double *qVal) {
    double randMode = GlobalResources::rand01();
    long actSelected;
    clock_t timeStart, timeEnd;
    bool tryAgain = true;
//cerr << "Rollout ";
//int z = 0;
    while (tryAgain) {
//cerr << "randMode-" << z << " : " << randMode << " " << pRollout[ROLLOUT_RANDHEURISTIC] << " " << pRollout[ROLLOUT_POL] << endl;
        if (randMode < pRollout[ROLLOUT_RANDHEURISTIC]) {
//cerr << "RandHeuristic\n";
            timeStart = clock();
            actSelected = belNode->getNxtActToTry();
//cerr << "actSelected: " << actSelected << endl;
            bool isTerm = model->getNextState(s, actSelected, immediateRew,
                    nxtSVals, obs);
//cerr << "nxtVal: " << nxtSVals[0] << " " << nxtSVals[1] << " " << *immediateRew << endl;
            if (isTerm) {
//cerr << "terminal\n";
                *qVal = startDisc * disc * model->getReward(nxtSVals);
            } else {
//cerr << "NotTerminal\n";
                model->solveHeuristic(nxtSVals, qVal);
                *qVal = startDisc * disc * (*qVal);
            }
            timeEnd = clock();
            rolloutUsed = ROLLOUT_RANDHEURISTIC;
            tryAgain = false;
//cerr << "Time randHeuristic: " << (timeEnd-timeStart) << " " << (timeEnd-timeStart)/(double) CLOCKS_PER_SEC << endl;
        } else {
//cerr << "PolHeuristic\n";
            timeStart = clock();
#if defined(DISTL1)
            BeliefNode *currNode = getNNBelNode(belNode);
#elif defined(DISTEMD)
            BeliefNode *currNode = getNNBelNode(belNode);
#endif
            if (currNode == nullptr) {
//cerr << " NOPOL\n";
                randMode = 0.0;
            } else {
//cerr << " YESPOL\n";
                //actSelected = currNode->getBestAct();
                actSelected = belNode->getNxtActToTry();
                model->getNextState(s, actSelected, immediateRew, nxtSVals,
                        obs);
                currNode = currNode->getChild(actSelected, obs);
                *qVal = startDisc * rolloutPolHelper(currNode, nxtSVals, disc);
                rolloutUsed = ROLLOUT_POL;
                tryAgain = false;
            }
            timeEnd = clock();
        }

//z++;
    }
    cRollout[rolloutUsed] = cRollout[rolloutUsed]
            + (timeEnd - timeStart) * 1000 / (double) CLOCKS_PER_SEC;
    nUsedRollout[rolloutUsed]++;

    return actSelected;
}

double Solver::rolloutPolHelper(BeliefNode *currNode, State &s, double disc) {
    if (currNode == nullptr) {
        // cerr << "nullptr in rolloutPolHelper!" << endl;
        return 0.0;
    } else if (currNode->nParticles == 0) {
        cerr << "nParticles == 0 in rolloutPolHelper" << endl;
    } else if (currNode->actChildren.size() == 0) {
        // cerr << "No children in rolloutPolHelper" << endl;
        return 0.0;
    }
    // || currNode->nParticles == 0 ||
    //        currNode->actChildren.size() == 0) {
    State nxtSVals;
    Observation obs;
    double immediateRew;
    //cout << currNode->nParticles << " " << currNode->particles.size() << endl;
    long actId = currNode->getBestAct();
    bool isTerm = model->getNextState(s, actId, &immediateRew, nxtSVals, obs);
    currNode = currNode->getChild(actId, obs);
    if (isTerm) {
        return immediateRew + disc * model->getReward(nxtSVals);
    } else {
        return immediateRew + disc * rolloutPolHelper(currNode, nxtSVals, disc);
    }
}
/*
 BeliefNode* Solver::getNNBelNodeEMD(BeliefNode *b) {
 cerr << "Enter getNNEMD ";
 double d, minDist;
 minDist = DBL_MAX;
 BeliefNode* nnBel = b->nnBel;
 //long nComp = 0;
 vector<BeliefNode*>::iterator itNode;
 for (itNode = policy->allNodes.begin(); itNode != policy->allNodes.end(); itNode++) {
 if ((*itNode)->tEmdSig < (*itNode)->tLastAddedParticle) {
 (*itNode)->setEMDSig();
 }
 if (b->tNNComp < (*itNode)->tLastAddedParticle) {
 if (b->tEmdSig < b->tLastAddedParticle) {
 b->setEMDSig();
 }
 d = cvCalcEMD2(b->emdSig, (*itNode)->emdSig, CV_DIST_L2);
 if (d < minDist) {
 minDist = d;
 nnBel = *itNode;
 }
 //nComp ++;
 }
 }
 //cerr << "tNNComp: " << b->tNNComp << " tLast " << b->tLastAddedParticle << " minDist: " << minDist << " aftComp: " << nComp << " from " << policy->allNodes.size() << endl;
 //cerr << "time: " << clock() << " " << BeliefNode::startTime << endl;
 b->tNNComp = (double) (clock() - BeliefNode::startTime)*10000 / CLOCKS_PER_SEC;
 //cerr << "new tNNComp: " << b->tNNComp << endl;
 b->nnBel = nnBel;
 cerr << "Done getNNEMD ";
 if (minDist > model->getDistTh()) { return nullptr; }
 return nnBel;
 }
 */
BeliefNode* Solver::getNNBelNode(BeliefNode *b) {
//cerr << "Enter getNNL1 ";
    double d, minDist;
    minDist = DBL_MAX;
    BeliefNode* nnBel = b->nnBel;
//long nComp = 0;
    vector<BeliefNode*>::iterator itNode;
    long nTry = 0;
    for (itNode = policy->allNodes.begin(); itNode != policy->allNodes.end();
            itNode++) {
        if (nTry < model->getMaxDistTry()) {
            if (b->tNNComp < (*itNode)->tLastAddedParticle) {
                d = b->distL1Independent(*itNode);
                if (d < minDist) {
                    minDist = d;
                    nnBel = *itNode;
                }
            }
            nTry++;
//nComp ++;
        } else {
            break;
        }
    }
//cerr << "tNNComp: " << b->tNNComp << " tLast " << b->tLastAddedParticle << " minDist: " << minDist << " aftComp: " << nComp << " from " << policy->allNodes.size() << endl;
//cerr << "time: " << clock() << " " << BeliefNode::startTime << endl;
    b->tNNComp = (double) (clock() - BeliefNode::startTime)
            * 1000/ CLOCKS_PER_SEC;
//cerr << "new tNNComp: " << b->tNNComp << endl;
    b->nnBel = nnBel;
//cerr << "Done getNNL1 ";
    if (minDist > model->getDistTh()) {
        return nullptr;
    }
    return nnBel;
}

void Solver::updWeightRolloutAct(double valImprovement) {
    /*
     cerr << "rolloutUsed: " << rolloutUsed << " calImprovement " << valImprovement/model->maxVal << " p " << pRollout[0] << " " << pRollout[1] << " w " << wRollout[0]
     << " " << wRollout[1] << " cost-0 " << nUsedRollout[0] << " " << cRollout[0] << " " << cRollout[0]/nUsedRollout[0] << " cost-1 "
     << nUsedRollout[1] << " " << cRollout[1] << " " << cRollout[1]/nUsedRollout[1] << endl;
     */
    if (valImprovement < 1e-7) {
        valImprovement = 0.0;
    }
    wRollout[rolloutUsed] = wRollout[rolloutUsed]
            * exp(
                    exploreCoef * (valImprovement / model->getMaxVal())
                            / (2 * pRollout[rolloutUsed]));
//cerr << "newW " << wRollout[0] << " " << wRollout[1] << endl;
    double totWRollout = 0.0;
    for (int i = 0; i < 2; i++) {
        totWRollout = totWRollout + wRollout[i];
    }
    double totP = 0.0;
    for (int i = 0; i < 2; i++) {
        pRollout[i] = ((1 - exploreCoef) * wRollout[i] / totWRollout
                + exploreCoef / 2) * nUsedRollout[i] / cRollout[i];
        totP = totP + pRollout[i];
    }
//cerr << "p " << pRollout[0] << " " << pRollout[1] << " totP: " << totP << " " << totWRollout << endl;
    for (int i = 0; i < 2; i++) {
        pRollout[i] = pRollout[i] / totP;
    }
//cerr << "normP " << pRollout[0] << " " << pRollout[1] << " totP: " << totP << " " << totWRollout << endl;
}

void Solver::backup(HistorySequence *history) {
//cerr << "In backup\n";
    vector<HistoryEntry*>::reverse_iterator itHist = history->histSeq.rbegin();
    double totRew;
    if ((*itHist)->actId == -1) {
        totRew = (*itHist)->qVal;
    } else {
        totRew = (*itHist)->qVal = (*itHist)->disc * (*itHist)->rew;
    }
    itHist++;
    for (; itHist != history->histSeq.rend(); itHist++) {
        if ((*itHist)->hasBeenBackup) {
            double prevTotRew = (*itHist)->qVal;
            totRew = (*itHist)->qVal = (*itHist)->disc * (*itHist)->rew
                    + totRew;
            (*itHist)->partOfBelNode->updateVal((*itHist)->actId, prevTotRew,
                    totRew, false);
        } else {
            totRew = (*itHist)->qVal = (*itHist)->disc * (*itHist)->rew
                    + totRew;
            (*itHist)->partOfBelNode->updateVal((*itHist)->actId, totRew);
            (*itHist)->hasBeenBackup = true;
        }
    }
}

double Solver::runSim(long nSteps, vector<long> &tChanges,
        vector<State> &trajSt, vector<long> &trajActId,
        vector<Observation> &trajObs, vector<double> &trajRew,
        long *actualNSteps, double *totChTime, double *totImpTime) {
    trajSt.clear();
    trajActId.clear();
    trajObs.clear();
    trajRew.clear();

    *totChTime = 0.0;
    *totImpTime = 0.0;
    clock_t chTimeStart, chTimeEnd, impSolTimeStart, impSolTimeEnd;
    *actualNSteps = nSteps;

    long maxTrials = model->getMaxTrials();
    double depthTh = model->getDepthTh();
    double discFactor = model->getDiscount();
    double currDiscFactor = 1.0;

    double rew;
    double val = 0.0;
    BeliefNode* currNode = policy->getRoot();
    BeliefNode* nxtNode;
    State currStVals, nxtStVals;
    model->sampleAnInitState(currStVals);
    cerr << "Initial State:" << endl;
    model->drawState(currStVals, cerr);
    trajSt.push_back(currStVals);

    vector<State> affectedRange;
    vector<ChangeType> typeOfChanges;
    //set<long> reachAffectedHistSeq; set<long> notReachAffectedHistSeq;
    set<HistorySequence*> affectedHistSeq;

    bool last = tChanges.empty();
    bool isTerm;
    vector<long>::iterator itCh = tChanges.begin();
    for (long i = 0; i < nSteps; i++) {
        cerr << "t-" << i << endl;
        if (!last && i == *itCh) {  // Model changes.
            cerr << "ModelChanges" << endl;
            //cerr << "Time-" << i << "\n";
            //model->drawEnv(cerr);
            //if (i == 10 || i == 20) {
            //  write(cerr);
            //}

            chTimeStart = clock();
            // Reset Affected data structures.
            affectedRange.clear();
            typeOfChanges.clear();
            //allHistories->resetAffected(reachAffectedHistSeq);
            //allHistories->resetAffected(notReachAffectedHistSeq);
            //reachAffectedHistSeq.clear(); notReachAffectedHistSeq.clear();
            resetAffected(affectedHistSeq);
            affectedHistSeq.clear();

            model->update(*itCh, affectedRange, typeOfChanges); // Add typeOfChanges
            identifyAffectedPol(affectedRange, typeOfChanges, affectedHistSeq);
            updatePol(affectedHistSeq);
            chTimeEnd = clock();
            *totChTime = *totChTime
                    + ((chTimeEnd - chTimeStart) * 1000 / CLOCKS_PER_SEC);
            /*
             #if defined(DELAFFECTEDSEQ)
             modifByDelAffectedSeq(currNode, affectedRange);
             #elif defined(DELAFFECTEDENTRIES)
             modifByDelAffectedEntries(currNode, affectedRange);
             #elif defined(DEFORMAFFECTED)
             modifByDeformAffected(currNode, affectedRange, typeOfChanges);
             #endif
             */
            itCh++;
            if (itCh == tChanges.end()) {
                last = true;
            }
        }
        impSolTimeStart = clock();
        improveSol(currNode, maxTrials, depthTh);
        impSolTimeEnd = clock();
        *totImpTime = *totImpTime
                + ((impSolTimeEnd - impSolTimeStart) * 1000 / CLOCKS_PER_SEC);

        isTerm = simAStep(currStVals, nxtStVals, &currNode, &nxtNode, &rew,
                trajSt, trajActId, trajObs);
        trajRew.push_back(rew);
        val = val + currDiscFactor * rew;
        currDiscFactor = currDiscFactor * discFactor;
        cerr << "Discount Factor: " << currDiscFactor << "; Total Reward: "
                << val << endl;
        if (isTerm) {
            *actualNSteps = i;
            break;
        }
        if (nxtNode == nullptr) {
            nxtNode = addChild(currNode, trajActId.back(), trajObs.back(), i);
        }
        currNode = nxtNode;
        currStVals = nxtStVals;
    }
    return val;
}

// To handle particle depletion.
BeliefNode* Solver::addChild(BeliefNode *currNode, long actId, Observation &obs,
        long timeStep) {
    cerr << "In add particle due to depletion" << endl;
    BeliefNode *nxtNode = nullptr;

    vector<State> partSt;
    vector<HistoryEntry*>::iterator it;
    for (it = currNode->particles.begin(); it != currNode->particles.end();
            it++) {
        partSt.push_back((*it)->st->state);
    }

    double disc = model->getDiscount();
    double d = pow(disc, timeStep);
    // Attempt to generate particles for next state based on the current belief,
    // the observation, and the action.
    vector<State> partNxtSt;
    model->getStatesSeeObs(actId, obs, partSt, partNxtSt);
    if (partNxtSt.empty()) {
        cerr << "Could not generate based on belief!" << endl;
        // If that fails, ignore the current belief.
        model->getStatesSeeObs(actId, obs, partNxtSt);
    }
    if (partNxtSt.empty()) {
        cerr << "Could not generate new particles!" << endl;
    }

    nxtNode = currNode->addChild(actId, obs);
    policy->allNodes.push_back(nxtNode);

    StateWrapper *st;
    HistoryEntry *histEntry;
    HistorySequence *histSeq;
    for (vector<State>::iterator itSt = partNxtSt.begin();
            itSt != partNxtSt.end(); itSt++) {
        st = allStates->add(*itSt);
        histEntry = new HistoryEntry(st);
        histSeq = new HistorySequence(histEntry, timeStep);
        nxtNode->add(histEntry);
        histEntry->partOfBelNode = nxtNode;
        allHistories->add(histSeq);

        // Assign value to the new history entry
        histEntry->rew = model->getReward(*itSt);
        histEntry->disc = d;
        if (model->isTerm(*itSt)) {
            histEntry->qVal = d * histEntry->rew;
        } else {
            histEntry->qVal = d
                    * (histEntry->rew + disc * model->getDefaultVal());
        }

        backup(histSeq);
    }
    return nxtNode;
}

void Solver::resetAffected(set<HistorySequence*> affectedHistSeq) {
    set<HistorySequence*>::iterator itHistSeq;
    for (itHistSeq = affectedHistSeq.begin();
            itHistSeq != affectedHistSeq.end(); itHistSeq++) {
        (*itHistSeq)->startAffectedIdx = LONG_MAX;
        (*itHistSeq)->endAffectedIdx = -1;
        (*itHistSeq)->chType = ChangeType::UNDEFINED;
    }
}

bool Solver::simAStep(State& currStVals, State &nxtStVals,
        BeliefNode **startNode, BeliefNode **nxtNode, double *rew,
        vector<State> &trajSt, vector<long> &trajActId,
        vector<Observation> &trajObs) {
    long actId = (*startNode)->getBestAct();
    Observation obs;
    bool isTerm = model->getNextState(currStVals, actId, rew, nxtStVals, obs);
    trajSt.push_back(nxtStVals);
    trajActId.push_back(actId);
    trajObs.push_back(obs);
    if (isTerm) {
        cerr << " Reach terminal" << endl;
        *rew = *rew + model->getDiscount() * model->getReward(nxtStVals);
        *nxtNode = nullptr;
    } else {
        *nxtNode = (*startNode)->getChild(actId, obs);
    }
    cerr << "Action: ";
    model->dispAct(actId, cerr);
    cerr << "; Reward: " << *rew << "; Obs: ";
    model->dispObs(obs, cerr);
    cerr << endl;
    model->drawState(nxtStVals, cerr);
    return isTerm;
}

void Solver::identifyAffectedPol(vector<State> &affectedRange,
        vector<ChangeType> &chTypes, set<HistorySequence*> &affectedHistSeq) {
    // Get affected states
    set<StateWrapper*> affectedStates;
    vector<State>::iterator it1, it2;
    vector<ChangeType>::iterator itType = chTypes.begin();
    it1 = affectedRange.begin();
    it2 = affectedRange.begin() + 1;
    /*
     cerr << "affRange: " << affectedRange.size() << endl;
     vector<State>::iterator itAff;
     for (itAff = affectedRange.begin(); itAff != affectedRange.end(); ) {
     cerr << "Aff fr " << (*itAff)[0] << " " << (*itAff)[1] << " to ";
     itAff ++;
     cerr << (*itAff)[0] << " " << (*itAff)[1] << "\n";
     itAff++;
     }
     */
    for (; itType != chTypes.end(); itType++) {
//cerr << "Affected: " << *itType << " from " << (*it1)[0] << " " << (*it1)[1] << " to " << (*it2)[0] << " " << (*it2)[1] <<  endl;
        allStates->identifyAffectedStates(*it1, *it2, *itType, affectedStates);
        it1++;
        it1++;
        it2++;
        it2++;
//cerr << "InSolver #affStates " << affectedStates.size() << endl;
    }
//cerr << "cool\n";
    // Get affected history seq.
    vector<HistoryEntry*>::iterator itH;
    set<StateWrapper*>::iterator itS;
    HistorySequence *histSeq;
//cerr << "last #affectedStates: " << affectedStates.size() << endl;
    for (itS = affectedStates.begin(); itS != affectedStates.end(); itS++) {
        for (itH = (*itS)->usedInHistoryEntries.begin();
                itH != (*itS)->usedInHistoryEntries.end(); itH++) {
            if (*itH != nullptr) {
                histSeq = allHistories->allHistSeq[(*itH)->seqId];

                // Set affected
//cerr << "InitAffectedIdx " << histSeq->startAffectedIdx << " " << histSeq->endAffectedIdx << " " << (*itH)->entryId << endl;
                if ((*itH)->entryId < histSeq->startAffectedIdx) {
                    histSeq->startAffectedIdx = (*itH)->entryId;
                }
                if ((*itH)->entryId > histSeq->endAffectedIdx) {
                    histSeq->endAffectedIdx = (*itH)->entryId;
                }
                histSeq->chType = max(histSeq->chType, (*itS)->chType);
                affectedHistSeq.insert(histSeq);
            }
            /*
             if (histSeq->endAffectedIdx >= histSeq->histSeq.size()) {
             cerr << "AftAffectedIdx  ID " << (*itH)->seqId << " "
             << (*itH)->entryId << " aff "
             << histSeq->startAffectedIdx << " "
             << histSeq->endAffectedIdx << " of "
             << histSeq->histSeq.size() << " affST " << (*itS)->s[0]
             << " " << (*itS)->s[1] << endl;
             for (unsigned long y = 0; y < histSeq->histSeq.size(); y++) {
             cerr << "El-" << y << " " << histSeq->histSeq[y]->st->s[0]
             << " " << histSeq->histSeq[y]->st->s[1] << " "
             << histSeq->histSeq[y]->entryId << endl;
             }
             }
             */
        }

        histSeq->chType = ChangeType::UNDEFINED;
    }
}

void Solver::updatePol(set<HistorySequence*> &affectedHistSeq) {
    double disc = model->getDiscount();
    set<HistorySequence*>::iterator itHistSeq;
    HistoryEntry *currHistEntry;
//cerr << "#affectedHistSeq: " << affectedHistSeq.size() << endl;
//int z = 0;
    for (itHistSeq = affectedHistSeq.begin();
            itHistSeq != affectedHistSeq.end(); itHistSeq++) {
//cerr << "Proc affected hist seq-" << z << " seqId " << (*itHistSeq)->id << endl;
        switch ((*itHistSeq)->chType) {
        case ChangeType::REWARD: {
//cerr << "endAffIdx: " << (*itHistSeq)->endAffectedIdx << " seq size: " << (*itHistSeq)->histSeq.size() << endl;
            if ((*itHistSeq)->endAffectedIdx
                    >= (long) (*itHistSeq)->histSeq.size()) {
                TextSerializer serializer(allStates);
                serializer.save(**itHistSeq, cerr);
            }
            currHistEntry = (*itHistSeq)->histSeq[(*itHistSeq)->endAffectedIdx];
//cerr << "LastAffected " << currHistEntry->st->s[0] << " " << currHistEntry->st->s[1] << endl;
            // Update last affected history entry.
            if ((*itHistSeq)->endAffectedIdx
                    == (long) (*itHistSeq)->histSeq.size() - 1) {
                long actSelected = GlobalResources::randIntBetween(0,
                        model->getNActions() - 1);
                State nxtSVals;
                Observation obs;
                double immediateRew;
                bool isTerm = model->getNextState(currHistEntry->st->state,
                        actSelected, &immediateRew, nxtSVals, obs);
                if (isTerm) {
//cerr << "in 0 " << currHistEntry->qVal << " becomes ";
                    currHistEntry->qVal =
                            currHistEntry->disc
                                    * (immediateRew
                                            + disc * model->getReward(nxtSVals));
//cerr << currHistEntry->qVal << endl;
                } else {
                    double nxtQVal;
                    model->solveHeuristic(nxtSVals, &nxtQVal);
//cerr << "in 1 " << currHistEntry->qVal << " becomes ";
                    currHistEntry->qVal = currHistEntry->disc
                            * (immediateRew + disc * nxtQVal);
//cerr << currHistEntry->qVal << endl;
                }
            } else {
//cerr << "in 2 " << currHistEntry->qVal << " becomes ";
                currHistEntry->rew = model->getReward(currHistEntry->st->state,
                        currHistEntry->actId);
                double prevRew = currHistEntry->qVal;
//cerr << "histSeq: " << (*itHistSeq)->histSeq.size() << " endAff " << (*itHistSeq)->endAffectedIdx << endl;
                currHistEntry->qVal =
                        currHistEntry->disc * currHistEntry->rew
                                + (*itHistSeq)->histSeq[(*itHistSeq)->endAffectedIdx
                                        + 1]->qVal;
                currHistEntry->partOfBelNode->updateVal(currHistEntry->actId,
                        prevRew, currHistEntry->qVal, false);
//cerr << currHistEntry->qVal << endl;
            }

            // Update the rest of the affected history entry.
            updateVal(*itHistSeq);

            break;
        }
        case ChangeType::ADDOBSTACLE: {
//cerr << "Start Working on addObs\n";
            vector<State> seqStVals, modifStSeq;
            vector<long> modifActSeq;
            vector<Observation> modifObsSeq;
            vector<double> modifRewSeq;
            vector<HistoryEntry*>::iterator itHistEntry;
            (*itHistSeq)->getStValsSeq(seqStVals);
            if ((*itHistSeq)->endAffectedIdx == (long) seqStVals.size() - 1) {
                if (model->modifStSeq(seqStVals, (*itHistSeq)->startAffectedIdx,
                        -1, modifStSeq, modifActSeq, modifObsSeq,
                        modifRewSeq)) {
                    removePathFrBelNode(*itHistSeq);
                    modifHistSeqFr(*itHistSeq, modifStSeq, modifActSeq,
                            modifObsSeq, modifRewSeq);
                }
            } else {
                if (model->modifStSeq(seqStVals, (*itHistSeq)->startAffectedIdx,
                        (*itHistSeq)->endAffectedIdx, modifStSeq, modifActSeq,
                        modifObsSeq, modifRewSeq)) {
                    removePathFrBelNode(*itHistSeq);
                    modifHistSeqFrTo(*itHistSeq, modifStSeq, modifActSeq,
                            modifObsSeq, modifRewSeq);
                }
            }

            // Update the rest of the affected history entry.
            //updateVal(*itHistSeq);
//cerr << "End working on addObs\n";
            break;
        }
        case ChangeType::ADDOBSERVATION:
        case ChangeType::ADDSTATE:
        case ChangeType::DELSTATE:
        case ChangeType::TRANSITION:
        case ChangeType::UNDEFINED:
            break;
        }
    }
}

void Solver::removePathFrBelNode(HistorySequence *history) {
    vector<HistoryEntry*>::iterator itHist;
    for (itHist = history->histSeq.begin() + 1;
            itHist != history->histSeq.end(); itHist++) {
        (*itHist)->partOfBelNode->updateVal((*itHist)->actId, (*itHist)->rew, 0,
                true);
        (*itHist)->hasBeenBackup = false;
    }
}

void Solver::modifHistSeqFr(HistorySequence *history, vector<State> &modifStSeq,
        vector<long> &modifActSeq, vector<Observation> &modifObsSeq,
        vector<double> &modifRewSeq) {
    vector<State>::iterator itSt = modifStSeq.begin();
    vector<long>::iterator itAct = modifActSeq.begin();
    vector<Observation>::iterator itObs = modifObsSeq.begin();
    vector<double>::iterator itRew = modifRewSeq.begin();

    BeliefNode *b = nullptr;
    StateWrapper *s;
    HistoryEntry *hEntry;

    long nOrgEntries = history->histSeq.size();
    /*
     if (history->id == 82458) {
     vector<HistoryEntry*>::iterator itH1;
     long hIdx1 = 0;
     cerr << "UNTIL END affected " << history->startAffectedIdx << " to " << history->endAffectedIdx << " from " << history->histSeq.size() << endl;
     for (itH1 = history->histSeq.begin(); itH1 != history->histSeq.end(); itH1++, hIdx1++) {
     cerr << "OrghistEntry-" << hIdx1 << " ";
     cerr << (*itH1)->st->s[0] << " " << (*itH1)->st->s[1];
     cerr << " act " << (*itH1)->actId;
     //cerr << " obs " << (*itH1)->obs[0] << " " << (*itH1)->obs[1];
     cerr << " rew " << (*itH1)->rew << endl;
     }
     }
     */
    long hIdx = history->startAffectedIdx - 1;
    double currDisc;
    if (itAct != modifActSeq.end()) {
        hEntry = history->histSeq[hIdx];
        currDisc = hEntry->disc;
        hEntry->actId = *itAct;
        hEntry->obs = *itObs;
        hEntry->rew = *itRew;
        b = hEntry->partOfBelNode->getChild(*itAct, *itObs);
        if (b == nullptr) {
//cerr << "ADDCHILD1\n";
            b = hEntry->partOfBelNode->addChild(*itAct, *itObs);
        }
        hIdx++;
        itSt++;
        itAct++;
        itObs++;
        itRew++;
        for (; itAct != modifActSeq.end();
                hIdx++, itSt++, itAct++, itObs++, itRew++) {
            currDisc = currDisc * model->getDiscount();
            if (hIdx < nOrgEntries) {
                hEntry = history->histSeq[hIdx];
                s = allStates->add(*itSt);
                hEntry->st = s;
                hEntry->actId = *itAct;
                hEntry->obs = *itObs;
                hEntry->rew = *itRew;
                hEntry->partOfBelNode = b;
                b->add(hEntry);
                hEntry->hasBeenBackup = false;
            } else {
                s = allStates->add(*itSt);
                hEntry = history->addEntry(s, *itAct, *itObs, *itRew, currDisc);
                hEntry->partOfBelNode = b;
                b->add(hEntry);
                hEntry->hasBeenBackup = false;
            }
            b = hEntry->partOfBelNode->getChild(*itAct, *itObs);
            if (b == nullptr) {
                /*
                 cerr << "ADDCHILD2 from " << (*itSt)[0] << " " << (*itSt)[1];
                 if (*itAct != -1) { cerr << " act " << *itAct;  }
                 cerr << "\n";
                 */
                b = hEntry->partOfBelNode->addChild(*itAct, *itObs);
            }
        }
        if (itSt != modifStSeq.end()) {
            currDisc = currDisc * model->getDiscount();
            if (hIdx < nOrgEntries) {
                hEntry = history->histSeq[hIdx];
                s = allStates->add(*itSt);
                hEntry->st = s;
                hEntry->actId = -1;
                hEntry->rew = *itRew;
                hEntry->partOfBelNode = b;
                b->add(hEntry);
                hEntry->hasBeenBackup = false;
            } else {
                s = allStates->add(*itSt);
                hEntry = history->addEntry(s, *itAct, *itObs, *itRew, currDisc);
                hEntry->partOfBelNode = b;
                b->add(hEntry);
                hEntry->hasBeenBackup = false;
            }
        }
    }

    hIdx++;
    vector<HistoryEntry*>::iterator itHistEntry;
    for (itHistEntry = history->histSeq.begin() + hIdx;
            itHistEntry != history->histSeq.end(); itHistEntry++) {
        (*itHistEntry)->st->delUsedInHistEntry(*itHistEntry);
    }
    history->histSeq.erase(history->histSeq.begin() + hIdx,
            history->histSeq.end());
    /*
     vector<HistoryEntry*>::iterator itH;
     hIdx = 0;
     cerr << "Size new hist: " << history->histSeq.size() << endl;
     for (itH = history->histSeq.begin(); itH != history->histSeq.end(); itH++, hIdx++) {
     cerr << "histEntry-" << hIdx << " " << (*itH)->st->s[0] << " " << (*itH)->st->s[1] << " act " << (*itH)->actId << endl;
     //" obs " << (*itH)->obs[0] << " " << (*itH)->obs[1] << " rew " << (*itH)->rew << endl;
     }
     cerr << "About to backup\n";
     */
    history->fixEntryId();
    backup(history);
//cerr << "After backup\n";
}

void Solver::modifHistSeqFrTo(HistorySequence *history,
        vector<State> &modifStSeq, vector<long> &modifActSeq,
        vector<Observation> &modifObsSeq, vector<double> &modifRewSeq) {

    vector<State>::iterator itSt = modifStSeq.begin();
    vector<long>::iterator itAct = modifActSeq.begin();
    vector<Observation>::iterator itObs = modifObsSeq.begin();
    vector<double>::iterator itRew = modifRewSeq.begin();

    BeliefNode *b;
    StateWrapper *s;
    HistoryEntry *hEntry;

    long nOrgEntries = history->endAffectedIdx;
    /*
     if (history->id == 82458) {
     vector<HistoryEntry*>::iterator itH1;
     long hIdx1 = 0;
     cerr << "UNTIL ENDIDX histSEq " << history->id << " affected " << history->startAffectedIdx << " to " << history->endAffectedIdx << " from " << history->histSeq.size() << endl;
     for (itH1 = history->histSeq.begin(); itH1 != history->histSeq.end(); itH1++, hIdx1++) {
     cerr << "OrghistEntry-" << hIdx1 << " ";
     cerr << (*itH1)->st->s[0] << " " << (*itH1)->st->s[1];
     cerr << " act " << (*itH1)->actId;
     //cerr << " obs " << (*itH1)->obs[0] << " " << (*itH1)->obs[1];
     cerr << " rew " << (*itH1)->rew << endl;
     }
     }
     */
    if (itAct != modifActSeq.end()) {
        long hIdx = history->startAffectedIdx - 1;
        hEntry = history->histSeq[hIdx];
        double currDisc = hEntry->disc;
        hEntry->actId = *itAct;
        hEntry->obs = *itObs;
        hEntry->rew = *itRew;
        b = hEntry->partOfBelNode->getChild(*itAct, *itObs);
        hEntry->hasBeenBackup = false;
        if (b == nullptr) {
            b = hEntry->partOfBelNode->addChild(*itAct, *itObs, hEntry);
        }
        hIdx++;
        itSt++;
        itAct++;
        itObs++;
        itRew++;
        for (; itAct != modifActSeq.end();
                hIdx++, itSt++, itAct++, itObs++, itRew++) {
            currDisc = model->getDiscount() * currDisc;
            if (hIdx <= nOrgEntries) {
                hEntry = history->histSeq[hIdx];
                s = allStates->add(*itSt);
                hEntry->st = s;
                hEntry->actId = *itAct;
                hEntry->obs = *itObs;
                hEntry->rew = *itRew;
                hEntry->partOfBelNode = b;
                b->add(hEntry);
                hEntry->hasBeenBackup = false;
            } else {
                s = allStates->add(*itSt);
                hEntry = history->addEntry(s, *itAct, *itObs, *itRew, currDisc,
                        hIdx);
                hEntry->partOfBelNode = b;
                b->add(hEntry);
                hEntry->hasBeenBackup = false;
            }
            b = hEntry->partOfBelNode->getChild(*itAct, *itObs);
            if (b == nullptr) {
                b = hEntry->partOfBelNode->addChild(*itAct, *itObs);
            }
        }

        if (hIdx <= nOrgEntries) {
            vector<HistoryEntry*>::iterator itHistEntry;
            for (itHistEntry = history->histSeq.begin() + hIdx;
                    itHistEntry != history->histSeq.begin() + nOrgEntries + 1;
                    itHistEntry++) {
                (*itHistEntry)->st->delUsedInHistEntry(*itHistEntry);
            }
            history->histSeq.erase(history->histSeq.begin() + hIdx,
                    history->histSeq.begin() + nOrgEntries + 1);
        }

        long sIdx = hIdx;
        vector<HistoryEntry*>::iterator itH;
        for (itH = history->histSeq.begin() + sIdx;
                itH != history->histSeq.end(); itH++, hIdx++) {
            currDisc = currDisc * model->getDiscount();
            (*itH)->disc = currDisc;
            (*itH)->partOfBelNode = b;
            b->add(*itH);
            (*itH)->hasBeenBackup = false;
            if ((*itH)->actId >= 0) {
                b = hEntry->partOfBelNode->getChild((*itH)->actId, (*itH)->obs);
                if (b == nullptr) {
                    b = hEntry->partOfBelNode->addChild((*itH)->actId,
                            (*itH)->obs);
                }
            }
        }
    } else {
        vector<HistoryEntry*>::iterator itHistEntry;
        for (itHistEntry = history->histSeq.begin() + history->startAffectedIdx;
                itHistEntry != history->histSeq.end(); itHistEntry++) {
            (*itHistEntry)->st->delUsedInHistEntry(*itHistEntry);
        }
        history->histSeq.erase(
                history->histSeq.begin() + history->startAffectedIdx,
                history->histSeq.end());
    }
    /*
     long hIdx = 0;
     cerr << "Size new hist: " << history->histSeq.size() << endl;
     for (vector<HistoryEntry*>::iterator itH = history->histSeq.begin(); itH != history->histSeq.end(); itH++, hIdx++) {
     cerr << "histEntry-" << hIdx << " " << (*itH)->st->s[0] << " " << (*itH)->st->s[1] << " act " << (*itH)->actId;
     //" obs " << (*itH)->obs[0] << " " << (*itH)->obs[1]
     cerr << " rew " << (*itH)->rew << endl;
     }
     cerr << "About to backup\n";
     */

    history->fixEntryId();
    backup(history);
//cerr << "After backup\n";
}

void Solver::updateVal(HistorySequence *histSeq) {
    double prevRew;
    long startAffectedIdx = histSeq->startAffectedIdx;
    long endAffectedIdx = histSeq->endAffectedIdx;
    double totRew = histSeq->histSeq[endAffectedIdx]->qVal;
    HistoryEntry *currHistEntry;
//cerr << "UpdateVal histSeq-" << histSeq->id << endl;
//cerr << "startAff: " << startAffectedIdx << " to " << endAffectedIdx << " curr totRew " << totRew << endl;
    for (long i = endAffectedIdx - 1; i >= startAffectedIdx; i--) {
        currHistEntry = histSeq->histSeq[i];
        prevRew = currHistEntry->qVal;
//cerr << "histEntry " << currHistEntry->st->s[0] << " " << currHistEntry->st->s[1] << " qVal " << currHistEntry->qVal << " ";
        currHistEntry->rew = model->getReward(currHistEntry->st->state,
                currHistEntry->actId);
        totRew = currHistEntry->qVal = currHistEntry->disc * currHistEntry->rew
                + totRew;
//cerr << " become " << currHistEntry->qVal << endl;
        currHistEntry->partOfBelNode->updateVal(currHistEntry->actId, prevRew,
                totRew, false);
    }

    for (long i = startAffectedIdx - 1; i >= 0; i--) {
        currHistEntry = histSeq->histSeq[i];
//cerr << "id-" << i << " histEntry " << currHistEntry->st->s[0] << " " << currHistEntry->st->s[1] << " qVal " << currHistEntry->qVal << " ";
        prevRew = currHistEntry->qVal;
        totRew = currHistEntry->qVal = currHistEntry->disc * currHistEntry->rew
                + totRew;
//cerr << " become " << currHistEntry->qVal << endl;
        currHistEntry->partOfBelNode->updateVal(currHistEntry->actId, prevRew,
                totRew, false);
    }
}

void Solver::improveSol(BeliefNode* startNode, long maxTrials, double depthTh) {
    double disc = model->getDiscount();
    vector<HistoryEntry*> samples;
    for (long i = 0; i < maxTrials; i++) {
        samples.push_back(startNode->sampleAParticle());
    }
    for (vector<HistoryEntry*>::iterator it = samples.begin();
            it != samples.end(); it++) {
        singleSearch(startNode, disc, depthTh, *it);
    }
}

void Solver::singleSearch(BeliefNode *startNode, double discount,
        double depthTh, HistoryEntry* startParticle) {
    HistorySequence *currHistSeq;
    long actIdx;
    double immediateRew;
    Observation obs;
    State nxtSVals;
    StateWrapper *nxtSt;

    double initStartVal = startNode->bestAvgQVal;
//cerr << "#part in startNode: " << startNode->nParticles << " " << initStartVal << endl;
    BeliefNode *currNode = startNode;
    BeliefNode *nxtNode;
    HistoryEntry *currHistEntry = new HistoryEntry(startParticle->st);
    currHistEntry->partOfBelNode = currNode;
    currNode->add(currHistEntry);
    long startDepth = allHistories->allHistSeq[startParticle->seqId]->startDepth
            + startParticle->entryId;
    double currDiscFactor = pow(discount, startDepth);
    currHistEntry->disc = currDiscFactor;
    currHistSeq = new HistorySequence(currHistEntry, startDepth);
    allHistories->add(currHistSeq);

    bool rolloutUsed = false;
    bool done = false;
    while (!done && currDiscFactor > depthTh) {
        if (model->getNActions() == currNode->actChildren.size()) {
            actIdx = currNode->getUCBAct();
            nxtSVals.vals.clear();
            obs.clear();
            done = model->getNextState(currHistEntry->st->state, actIdx,
                    &immediateRew, nxtSVals, obs);
            currHistEntry->rew = immediateRew;
            nxtSt = allStates->add(nxtSVals);
            currHistEntry = currHistSeq->addEntry(actIdx, obs, nxtSt);

            currDiscFactor = currDiscFactor * discount;
            currHistEntry->disc = currDiscFactor;
            nxtNode = currNode->addChild(actIdx, obs, currHistEntry);
            //nxtNode->setEMDSig(model->nParticles, model->nStVars);
            policy->allNodes.push_back(nxtNode);
            currHistEntry->partOfBelNode = nxtNode;
//nxtNode->add(currHistEntry);
            nxtSt->addInfo(currHistEntry);
            nxtSt->addInfo(nxtNode);
            currNode = nxtNode;
            if (done) {
                currHistEntry->rew = model->getReward(nxtSVals);
                currHistEntry->qVal = currHistEntry->disc * currHistEntry->rew;
            }
        } else {
            double qVal;
            nxtSVals.vals.clear();
            obs.clear();
            actIdx = getRolloutAct(currNode, currHistEntry->st->state,
                    currDiscFactor, discount, nxtSVals, obs, &immediateRew,
                    &qVal);
//cerr << "getRolloutImmediateRew: " << immediateRew << endl;
            currHistEntry->rew = immediateRew;
            nxtSt = allStates->add(nxtSVals);
            currHistEntry = currHistSeq->addEntry(actIdx, obs, nxtSt);
            nxtNode = currNode->addChild(actIdx, obs, currHistEntry);
            //nxtNode->setEMDSig(model->nParticles, model->nStVars);
            policy->allNodes.push_back(nxtNode);
            currHistEntry->partOfBelNode = nxtNode;
//nxtNode->add(currHistEntry);
            currHistEntry->disc = currDiscFactor * discount;
            currHistEntry->qVal = qVal;
            nxtSt->addInfo(currHistEntry);
            nxtSt->addInfo(nxtNode);

            rolloutUsed = true;
            done = true;
        }
    }

    backup(currHistSeq);
    if (rolloutUsed) {
//cerr << "currAvgVal: " << startNode->bestAvgQVal << " " << initStartVal << " #part " << startNode->nParticles  << endl;
        updWeightRolloutAct(startNode->bestAvgQVal - initStartVal);
    }
    rolloutUsed = false;
}

void Solver::write(ostream &os) {
    TextSerializer serializer(allStates);
    serializer.save(*allStates, os);
    serializer.save(*allHistories, os);
    os << "BELIEFTREE-BEGIN\n";
    policy->write(os);
    os << "BELIEFTREE-END\n";
}
