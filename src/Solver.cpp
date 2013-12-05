#include "Solver.hpp"

#include <cfloat>                       // for DBL_MAX
#include <climits>                      // for LONG_MAX
#include <cmath>                        // for pow, exp
#include <ctime>                        // for clock, clock_t, CLOCKS_PER_SEC

#include <algorithm>                    // for max
#include <iostream>                     // for operator<<, cerr, ostream, basic_ostream, endl, basic_ostream<>::__ostream_type, cout
#include <memory>                       // for unique_ptr
#include <set>                          // for _Rb_tree_const_iterator, set, std::set<>::iterator
#include <tuple>                        // for tie
#include <utility>                      // for pair, make_pair, _Rb_tree_const_iterator, set, std::set<>::iterator
#include <vector>                       // for vector, std::vector<>::iterator, std::vector<>::reverse_iterator

#include "defs.hpp"                     // for RandomGenerator
#include "BeliefNode.hpp"               // for BeliefNode, BeliefNode::startTime
#include "BeliefTree.hpp"               // for BeliefTree
#include "ChangeType.hpp"               // for ChangeType
#include "Histories.hpp"                // for Histories
#include "HistoryEntry.hpp"             // for HistoryEntry
#include "HistorySequence.hpp"          // for HistorySequence
#include "Model.hpp"                    // for Model
#include "Observation.hpp"              // for Observation
#include "State.hpp"                    // for State
#include "StatePool.hpp"                // for StatePool
#include "StateInfo.hpp"                // for StateInfo
#include "TextSerializer.hpp"           // for TextSerializer

using std::cerr;
using std::cout;
using std::endl;

Solver::Solver(RandomGenerator *randGen, Model *model) :
    randGen(randGen),
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

Solver::~Solver() {
    delete allStates;
    delete allHistories;
    delete policy;
}

void Solver::genPol(long maxTrials, double depthTh) {
    double disc = model->getDiscount();
    BeliefNode *root = policy->getRoot();

    // Set root to have at least one particle for each possible action.
    for (unsigned long action = 0; action < model->getNActions(); action++) {
        std::unique_ptr<State> statePtr = model->sampleAnInitState();
        StateInfo *stateInfo = allStates->add(std::move(statePtr));
        State *state = stateInfo->getState();

        HistoryEntry *rootHistEntry = new HistoryEntry(stateInfo);
        rootHistEntry->partOfBelNode = root;
        root->add(rootHistEntry);
        stateInfo->addHistoryEntry(rootHistEntry);
        stateInfo->addBeliefNode(root);
        HistorySequence *currHistSeq = new HistorySequence(rootHistEntry, 0);
        allHistories->add(currHistSeq);

        // Use the generative model to step forward.
        Model::StepResult result = model->generateStep(*state, action);
        State *nextState = result.nextState.get();
        stateInfo = allStates->add(std::move(result.nextState));

        rootHistEntry->discount = 1.0;
        rootHistEntry->immediateReward = result.immediateReward;

        HistoryEntry *currHistEntry = currHistSeq->addEntry(action,
                                      result.observation, stateInfo);
        BeliefNode *currNode = root->addChild(action, result.observation,
                                              currHistEntry);

        policy->allNodes.push_back(currNode);
        currHistEntry->partOfBelNode = currNode;
        currHistEntry->discount = disc;
        currHistEntry->immediateReward = 0.0;
        stateInfo->addHistoryEntry(currHistEntry);
        stateInfo->addBeliefNode(currNode);

        if (result.isTerminal) {
            currHistEntry->immediateReward = model->getReward(*nextState);
            currHistEntry->qVal = disc * currHistEntry->immediateReward;
        } else {
            currHistEntry->immediateReward = model->getReward(*nextState);
            currHistEntry->qVal = disc * currHistEntry->immediateReward;
        }

        rootHistEntry->qVal = result.immediateReward + currHistEntry->qVal;
        root->updateVal(action, rootHistEntry->qVal);
    }

    // Start expanding the tree.
    for (long i = 0; i < maxTrials; i++) {
        singleSearch(disc, depthTh);
    }
}

void Solver::singleSearch(double discountFactor, double depthTh) {
    StateInfo *stateInfo = allStates->add(model->sampleAnInitState());
    singleSearch(policy->getRoot(), stateInfo, 0, discountFactor, depthTh);
}

void Solver::singleSearch(BeliefNode *startNode, StateInfo *startStateInfo,
                          long startDepth, double discountFactor, double depthTh) {
    double initialStartNodeQVal = startNode->bestAvgQVal;

    BeliefNode *currNode = startNode;
    double currentDiscount = std::pow(discountFactor, startDepth);
    HistoryEntry *currHistEntry = new HistoryEntry(startStateInfo);
    currHistEntry->partOfBelNode = currNode;
    currNode->add(currHistEntry);
    currHistEntry->discount = currentDiscount;

    HistorySequence *currHistSeq = new HistorySequence(currHistEntry,
            startDepth);
    allHistories->add(currHistSeq);

    bool rolloutUsed = false;
    bool done = false;
    while (!done && currentDiscount > depthTh) {
        Model::StepResult result;
        double qVal = 0;
        if (model->getNActions() == currNode->actChildren.size()) {
            // If all actions have been attempted, use UCB
            long action = currNode->getUCBAct();
            result = model->generateStep(*currHistEntry->stateInfo->getState(),
                    action);
            done = result.isTerminal;
        } else {
            // Otherwise use the rollout method
            std::tie(result, qVal) = getRolloutAction(currNode,
                            *currHistEntry->stateInfo->getState(),
                            currentDiscount, discountFactor);
            rolloutUsed = true;
            done = true;
        }
        currHistEntry->immediateReward = result.immediateReward;
        StateInfo *nextStateInfo = allStates->add(std::move(result.nextState));
        // Ownership of the state has been transferred to the stateInfo.
        currHistEntry = currHistSeq->addEntry(result.action, result.observation,
                                              nextStateInfo);
        currentDiscount *= discountFactor;
        currHistEntry->discount = currentDiscount;

        BeliefNode *nextNode = currNode->addChild(result.action,
                               result.observation, currHistEntry);
        policy->allNodes.push_back(nextNode);
        currHistEntry->partOfBelNode = nextNode;

        nextStateInfo->addHistoryEntry(currHistEntry);
        nextStateInfo->addBeliefNode(nextNode);

        if (rolloutUsed) {
            currHistEntry->qVal = qVal;
        } else {
            if (done) {
                currHistEntry->immediateReward = model->getReward(
                                                     *nextStateInfo->getState());
                currHistEntry->qVal = currHistEntry->discount
                                      * currHistEntry->immediateReward;
            }
        }
    }
    backup(currHistSeq);
    if (rolloutUsed) {
        updWeightRolloutAct(policy->root->bestAvgQVal - initialStartNodeQVal);
    }
    rolloutUsed = false;
}

void Solver::backup(HistorySequence *history) {
    std::vector<HistoryEntry *>::reverse_iterator itHist =
        history->histSeq.rbegin();
    double totRew;
    if ((*itHist)->actId == -1) {
        totRew = (*itHist)->qVal;
    } else {
        totRew = (*itHist)->qVal = (*itHist)->discount
                                   * (*itHist)->immediateReward;
    }
    itHist++;
    for (; itHist != history->histSeq.rend(); itHist++) {
        if ((*itHist)->hasBeenBackup) {
            double prevTotRew = (*itHist)->qVal;
            totRew = (*itHist)->qVal = (*itHist)->discount
                                       * (*itHist)->immediateReward + totRew;
            (*itHist)->partOfBelNode->updateVal((*itHist)->actId, prevTotRew,
                                                totRew, false);
        } else {
            totRew = (*itHist)->qVal = (*itHist)->discount
                                       * (*itHist)->immediateReward + totRew;
            (*itHist)->partOfBelNode->updateVal((*itHist)->actId, totRew);
            (*itHist)->hasBeenBackup = true;
        }
    }
}

std::pair<Model::StepResult, double> Solver::getRolloutAction(
    BeliefNode *belNode, State &state, double startDisc, double disc) {
    long action;
    Model::StepResult result;
    double qVal;

    if (std::bernoulli_distribution(pRollout[ROLLOUT_RANDHEURISTIC])(*randGen)) {
        rolloutUsed = ROLLOUT_RANDHEURISTIC;
    } else {
        rolloutUsed = ROLLOUT_POL;
    }
    std::clock_t startTime, endTime;
    if (rolloutUsed == ROLLOUT_POL) {
        startTime = std::clock();
#if defined(DISTL1)
        BeliefNode *currNode = getNNBelNode(belNode);
#elif defined(DISTEMD)
        BeliefNode *currNode = getNNBelNode(belNode);
#endif
        if (currNode == nullptr) {
            rolloutUsed = ROLLOUT_RANDHEURISTIC; // Use RANDHEURISTIC instead.
        } else {
            action = belNode->getNxtActToTry();
            result = model->generateStep(state, action);
            currNode = currNode->getChild(action, result.observation);
            qVal = (startDisc
                    * rolloutPolHelper(currNode, *result.nextState, disc));
            rolloutUsed = ROLLOUT_POL;
            endTime = std::clock();
        }
    }
    if (rolloutUsed == ROLLOUT_RANDHEURISTIC) {
        startTime = std::clock();
        action = belNode->getNxtActToTry();
        result = model->generateStep(state, action);
        if (result.isTerminal) {
            qVal = model->getReward(*result.nextState);
        } else {
            qVal = model->solveHeuristic(*result.nextState);
        }
        qVal *= startDisc * disc;
        rolloutUsed = ROLLOUT_RANDHEURISTIC;
        endTime = std::clock();
    }
    cRollout[rolloutUsed] += (endTime - startTime) * 1000.0 / CLOCKS_PER_SEC;
    nUsedRollout[rolloutUsed]++;

    return std::make_pair(std::move(result), qVal);
}

double Solver::rolloutPolHelper(BeliefNode *currNode, State &state,
                                double disc) {
    if (currNode == nullptr) {
        // cerr << "nullptr in rolloutPolHelper!" << endl;
        return 0.0;
    } else if (currNode->nParticles == 0) {
        cerr << "nParticles == 0 in rolloutPolHelper" << endl;
        return 0.0;
    } else if (currNode->actChildren.size() == 0) {
        // cerr << "No children in rolloutPolHelper" << endl;
        return 0.0;
    }

    long action = currNode->getBestAct();
    Model::StepResult result = model->generateStep(state, action);
    currNode = currNode->getChild(action, result.observation);
    double qVal = result.immediateReward;
    if (result.isTerminal) {
        qVal += disc * model->getReward(*result.nextState);
    } else {
        qVal += disc * rolloutPolHelper(currNode, *result.nextState, disc);
    }
    return qVal;
}

BeliefNode *Solver::getNNBelNode(BeliefNode *b) {
    double d, minDist;
    minDist = DBL_MAX;
    BeliefNode *nnBel = b->nnBel;
    std::vector<BeliefNode *>::iterator itNode;
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
        } else {
            break;
        }
    }
    b->tNNComp = (double) (std::clock() - BeliefNode::startTime)
                 * 1000/ CLOCKS_PER_SEC;
    b->nnBel = nnBel;
    if (minDist > model->getDistTh()) {
        return nullptr;
    }
    return nnBel;
}

void Solver::updWeightRolloutAct(double valImprovement) {
    if (valImprovement < 1e-7) {
        valImprovement = 0.0;
    }
    wRollout[rolloutUsed] = wRollout[rolloutUsed]
                            * std::exp(
                                exploreCoef * (valImprovement / model->getMaxVal())
                                / (2 * pRollout[rolloutUsed]));
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
    for (int i = 0; i < 2; i++) {
        pRollout[i] = pRollout[i] / totP;
    }
}

double Solver::runSim(long nSteps, std::vector<long> &changeTimes,
                      std::vector<std::unique_ptr<State>> &trajSt,
                      std::vector<long> &trajActId, std::vector<Observation> &trajObs,
                      std::vector<double> &trajRew, long *actualNSteps, double *totChTime,
                      double *totImpTime) {
    trajSt.clear();
    trajActId.clear();
    trajObs.clear();
    trajRew.clear();

    *totChTime = 0.0;
    *totImpTime = 0.0;
    std::clock_t chTimeStart, chTimeEnd, impSolTimeStart, impSolTimeEnd;
    *actualNSteps = nSteps;
    long maxTrials = model->getMaxTrials();
    double depthTh = model->getDepthTh();
    double discFactor = model->getDiscount();
    double currDiscFactor = 1.0;
    double discountedTotalReward = 0.0;

    BeliefNode *currNode = policy->getRoot();
    std::unique_ptr<State> state(model->sampleAnInitState());
    State *currentState = state.get();
    trajSt.push_back(std::move(state));

    cerr << "Initial State:" << endl;
    model->drawState(*currentState, cerr);

    bool changesDone = changeTimes.empty();
    std::vector<long>::iterator itCh = changeTimes.begin();
    for (long i = 0; i < nSteps; i++) {
        cerr << "t-" << i << endl;
        if (!changesDone && i == *itCh) {  // Model changes.
            cerr << "ModelChanges" << endl;

            chTimeStart = std::clock();

            std::set<HistorySequence *> affectedHistSeq;
            std::vector<std::unique_ptr<State> > affectedRange;
            std::vector<ChangeType> typeOfChanges;
            model->update(*itCh, &affectedRange, &typeOfChanges); // Add typeOfChanges
            identifyAffectedPol(affectedRange, typeOfChanges, affectedHistSeq);
            updatePol(affectedHistSeq);
            resetAffected(affectedHistSeq);

            chTimeEnd = std::clock();
            *totChTime = *totChTime
                         + ((chTimeEnd - chTimeStart) * 1000 / CLOCKS_PER_SEC);

            itCh++;
            if (itCh == changeTimes.end()) {
                changesDone = true;
            }
        }
        impSolTimeStart = std::clock();
        improveSol(currNode, maxTrials, depthTh);
        impSolTimeEnd = std::clock();
        *totImpTime = *totImpTime
                      + ((impSolTimeEnd - impSolTimeStart) * 1000 / CLOCKS_PER_SEC);

        Model::StepResult result = simAStep(currNode, *currentState);
        currentState = result.nextState.get();

        trajActId.push_back(result.action);
        trajObs.push_back(result.observation);
        trajSt.push_back(std::move(result.nextState)); // trajSt is responsible for ownership
        trajRew.push_back(result.immediateReward);
        discountedTotalReward += currDiscFactor * result.immediateReward;
        currDiscFactor = currDiscFactor * discFactor;
        cerr << "Discount Factor: " << currDiscFactor << "; Total Reward: "
             << discountedTotalReward << endl;
        if (result.isTerminal) {
            *actualNSteps = i;
            break;
        }

        BeliefNode *nextNode = currNode->getChild(result.action,
                               result.observation);
        if (nextNode == nullptr) {
            nextNode = addChild(currNode, trajActId.back(), trajObs.back(), i);
        }
        currNode = nextNode;
    }
    return discountedTotalReward;
}

Model::StepResult Solver::simAStep(BeliefNode *currentBelief,
                                   State &currentState) {
    long action = currentBelief->getBestAct();
    Model::StepResult result = model->generateStep(currentState, action);
    if (result.isTerminal) {
        cerr << " Reach terminal" << endl;
        result.immediateReward += model->getDiscount()
                                  * model->getReward(*result.nextState);
    }
    cerr << "Action: ";
    model->dispAct(action, cerr);
    cerr << "; Reward: " << result.immediateReward << "; Obs: ";
    model->dispObs(result.observation, cerr);
    cerr << endl;
    model->drawState(*result.nextState, cerr);
    return result;
}

void Solver::improveSol(BeliefNode *startNode, long maxTrials, double depthTh) {
    double disc = model->getDiscount();
    std::vector<StateInfo *> samples;
    HistoryEntry *entry = startNode->sampleAParticle(randGen);
    long depth = entry->getId()
                 + allHistories->allHistSeq[entry->getSeqId()]->startDepth;
    samples.push_back(entry->stateInfo);
    for (long i = 0; i < maxTrials - 1; i++) {
        samples.push_back(startNode->sampleAParticle(randGen)->stateInfo);
    }
    for (StateInfo *sample : samples) {
        singleSearch(startNode, sample, depth, disc, depthTh);
    }
}

BeliefNode *Solver::addChild(BeliefNode *currNode, long actId, Observation &obs,
                             long timeStep) {
    cerr << "In add particle due to depletion" << endl;
    BeliefNode *nxtNode = nullptr;

    std::vector<State *> particles;
    std::vector<HistoryEntry *>::iterator it;
    for (HistoryEntry *entry : currNode->particles) {
        particles.push_back(entry->stateInfo->getState());
    }

    double disc = model->getDiscount();
    double currentDiscount = std::pow(disc, timeStep);
// Attempt to generate particles for next state based on the current belief,
// the observation, and the action.
    std::vector<std::unique_ptr<State> > nextParticles(
        model->generateParticles(actId, obs, particles));
    if (nextParticles.empty()) {
        cerr << "Could not generate based on belief!" << endl;
        // If that fails, ignore the current belief.
        nextParticles = model->generateParticles(actId, obs);
    }
    if (nextParticles.empty()) {
        cerr << "Could not generate new particles!" << endl;
    }

    nxtNode = currNode->addChild(actId, obs);
    policy->allNodes.push_back(nxtNode);

    for (std::unique_ptr<State> &uniqueStatePtr : nextParticles) {
        State *state = uniqueStatePtr.get();
        StateInfo *stateInfo = allStates->add(std::move(uniqueStatePtr));
        HistoryEntry *histEntry = new HistoryEntry(stateInfo);
        HistorySequence *histSeq = new HistorySequence(histEntry, timeStep);
        nxtNode->add(histEntry);
        histEntry->partOfBelNode = nxtNode;
        allHistories->add(histSeq);

        // Assign value to the new history entry
        histEntry->immediateReward = model->getReward(*state);
        histEntry->discount = currentDiscount;
        if (model->isTerm(*state)) {
            histEntry->qVal = currentDiscount * histEntry->immediateReward;
        } else {
            histEntry->qVal = currentDiscount
                              * (histEntry->immediateReward
                                 + disc * model->getDefaultVal());
        }
        backup(histSeq);
    }
    return nxtNode;
}


void Solver::resetAffected(std::set<HistorySequence *> affectedHistSeq) {
    std::set<HistorySequence *>::iterator itHistSeq;
    for (itHistSeq = affectedHistSeq.begin();
            itHistSeq != affectedHistSeq.end(); itHistSeq++) {
        (*itHistSeq)->startAffectedIdx = LONG_MAX;
        (*itHistSeq)->endAffectedIdx = -1;
        (*itHistSeq)->changeType = ChangeType::UNDEFINED;
    }
}

void Solver::identifyAffectedPol(
    std::vector<std::unique_ptr<State> > &affectedRange,
    std::vector<ChangeType> &chTypes,
    std::set<HistorySequence *> &affectedHistSeq) {
    // Get affected states
    std::set<StateInfo *> affectedStates;
    std::vector<std::unique_ptr<State> >::iterator it1, it2;
    std::vector<ChangeType>::iterator itType = chTypes.begin();
    it1 = affectedRange.begin();
    it2 = affectedRange.begin() + 1;
    for (; itType != chTypes.end(); itType++) {
        allStates->identifyAffectedStates(**it1, **it2, *itType,
                                          affectedStates);
        it1++;
        it1++;
        it2++;
        it2++;
    }
    std::vector<HistoryEntry *>::iterator itH;
    std::set<StateInfo *>::iterator itS;
    HistorySequence *histSeq;
    for (itS = affectedStates.begin(); itS != affectedStates.end(); itS++) {
        for (itH = (*itS)->usedInHistoryEntries.begin();
                itH != (*itS)->usedInHistoryEntries.end(); itH++) {
            if (*itH != nullptr) {
                histSeq = allHistories->allHistSeq[(*itH)->seqId];

                // Set affected
                if ((*itH)->entryId < histSeq->startAffectedIdx) {
                    histSeq->startAffectedIdx = (*itH)->entryId;
                }
                if ((*itH)->entryId > histSeq->endAffectedIdx) {
                    histSeq->endAffectedIdx = (*itH)->entryId;
                }
                histSeq->changeType = std::max(histSeq->changeType, (*itS)->chType);
                affectedHistSeq.insert(histSeq);
            }
        }

        histSeq->changeType = ChangeType::UNDEFINED;
    }
}

void Solver::updatePol(std::set<HistorySequence *> &affectedHistSeq) {
    double discountFactor = model->getDiscount();
    for (HistorySequence *sequence : affectedHistSeq) {
        HistoryEntry *currHistEntry;
        switch (sequence->changeType) {
        case ChangeType::REWARD: {
//cerr << "endAffIdx: " << (*itHistSeq)->endAffectedIdx << " seq size: " << (*itHistSeq)->histSeq.size() << endl;
//            if ((*itHistSeq)->endAffectedIdx
//                    >= (long) (*itHistSeq)->histSeq.size()) {
//                TextSerializer serializer(this);
//                serializer.save(**itHistSeq, cerr);
//            }
            currHistEntry = sequence->histSeq[sequence->endAffectedIdx];
            if (sequence->endAffectedIdx
                    == (long) sequence->histSeq.size() - 1) {
                long actSelected = std::uniform_int_distribution<long>(0,
                        model->getNActions() - 1)(*randGen);
                Model::StepResult result = model->generateStep(
                                               *currHistEntry->stateInfo->getState(), actSelected);
                if (result.isTerminal) {
                    currHistEntry->qVal = (currHistEntry->discount
                                           * (result.immediateReward
                                              + discountFactor
                                              * model->getReward(
                                                  *result.nextState)));
                } else {
                    double nxtQVal = model->solveHeuristic(*result.nextState);
                    currHistEntry->qVal =
                        (currHistEntry->discount
                         * (result.immediateReward
                            + discountFactor * nxtQVal));
                }
            } else {
                currHistEntry->immediateReward = model->getReward(
                                                     *currHistEntry->stateInfo->getState(),
                                                     currHistEntry->actId);
                double prevRew = currHistEntry->qVal;
                currHistEntry->qVal =
                    (currHistEntry->discount
                     * currHistEntry->immediateReward
                     + sequence->histSeq[sequence->endAffectedIdx
                                         + 1]->qVal);
                currHistEntry->partOfBelNode->updateVal(currHistEntry->actId,
                                                        prevRew, currHistEntry->qVal, false);
            }
            updateVal(sequence);
            break;
        }
        case ChangeType::ADDOBSTACLE: {
            std::vector<State const *> states = sequence->getStates();
            std::vector<std::unique_ptr<State>> modifStSeq;
            std::vector<long> modifActSeq;
            std::vector<Observation> modifObsSeq;
            std::vector<double> modifRewSeq;
            if (sequence->endAffectedIdx == (long) states.size() - 1) {
                if (model->modifStSeq(states, sequence->startAffectedIdx,
                                      -1, &modifStSeq, &modifActSeq, &modifObsSeq,
                                      &modifRewSeq)) {
                    removePathFrBelNode(sequence);
                    modifHistSeqFr(sequence, modifStSeq, modifActSeq,
                                   modifObsSeq, modifRewSeq);
                }
            } else {
                if (model->modifStSeq(states, sequence->startAffectedIdx,
                                      sequence->endAffectedIdx, &modifStSeq, &modifActSeq,
                                      &modifObsSeq, &modifRewSeq)) {
                    removePathFrBelNode(sequence);
                    modifHistSeqFrTo(sequence, modifStSeq, modifActSeq,
                                     modifObsSeq, modifRewSeq);
                }
            }

            // Update the rest of the affected history entry.
//            updateVal(*itHistSeq);
            break;
        }
        // These cases have not been implemented!!
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
    std::vector<HistoryEntry *>::iterator itHist;
    for (itHist = history->histSeq.begin() + 1;
            itHist != history->histSeq.end(); itHist++) {
        (*itHist)->partOfBelNode->updateVal((*itHist)->actId,
                                            (*itHist)->immediateReward, 0, true);
        (*itHist)->hasBeenBackup = false;
    }
}

void Solver::modifHistSeqFr(HistorySequence *history,
                            std::vector<std::unique_ptr<State> > &modifStSeq, std::vector<long> &modifActSeq,
                            std::vector<Observation> &modifObsSeq,
                            std::vector<double> &modifRewSeq) {

    std::vector<std::unique_ptr<State> >::iterator itSt = modifStSeq.begin();
    std::vector<long>::iterator itAct = modifActSeq.begin();
    std::vector<Observation>::iterator itObs = modifObsSeq.begin();
    std::vector<double>::iterator itRew = modifRewSeq.begin();

    BeliefNode *b = nullptr;
    StateInfo *s;
    HistoryEntry *hEntry;

    long nOrgEntries = history->histSeq.size();
    /*
     if (history->id == 82458) {
     std::vector<HistoryEntry *>::iterator itH1;
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
        currDisc = hEntry->discount;
        hEntry->actId = *itAct;
        hEntry->obs = *itObs;
        hEntry->immediateReward = *itRew;
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
                s = allStates->add(std::move(*itSt));
                hEntry->stateInfo = s;
                hEntry->actId = *itAct;
                hEntry->obs = *itObs;
                hEntry->immediateReward = *itRew;
                hEntry->partOfBelNode = b;
                b->add(hEntry);
                hEntry->hasBeenBackup = false;
            } else {
                s = allStates->add(std::move(*itSt));
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
                s = allStates->add(std::move(*itSt));
                hEntry->stateInfo = s;
                hEntry->actId = -1;
                hEntry->immediateReward = *itRew;
                hEntry->partOfBelNode = b;
                b->add(hEntry);
                hEntry->hasBeenBackup = false;
            } else {
                s = allStates->add(std::move(*itSt));
                hEntry = history->addEntry(s, *itAct, *itObs, *itRew, currDisc);
                hEntry->partOfBelNode = b;
                b->add(hEntry);
                hEntry->hasBeenBackup = false;
            }
        }
    }

    hIdx++;
    std::vector<HistoryEntry *>::iterator itHistEntry;
    for (itHistEntry = history->histSeq.begin() + hIdx;
            itHistEntry != history->histSeq.end(); itHistEntry++) {
        (*itHistEntry)->stateInfo->delUsedInHistEntry(*itHistEntry);
    }
    history->histSeq.erase(history->histSeq.begin() + hIdx,
                           history->histSeq.end());
    /*
     std::vector<HistoryEntry *>::iterator itH;
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
                              std::vector<std::unique_ptr<State> > &modifStSeq,
                              std::vector<long> &modifActSeq, std::vector<Observation> &modifObsSeq,
                              std::vector<double> &modifRewSeq) {

    std::vector<std::unique_ptr<State> >::iterator itSt = modifStSeq.begin();
    std::vector<long>::iterator itAct = modifActSeq.begin();
    std::vector<Observation>::iterator itObs = modifObsSeq.begin();
    std::vector<double>::iterator itRew = modifRewSeq.begin();

    BeliefNode *b = nullptr;
    StateInfo *s;
    HistoryEntry *hEntry;

    long nOrgEntries = history->endAffectedIdx;

    if (itAct != modifActSeq.end()) {
        long hIdx = history->startAffectedIdx - 1;
        hEntry = history->histSeq[hIdx];
        double currDisc = hEntry->discount;
        hEntry->actId = *itAct;
        hEntry->obs = *itObs;
        hEntry->immediateReward = *itRew;
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
                s = allStates->add(std::move(*itSt));
                hEntry->stateInfo = s;
                hEntry->actId = *itAct;
                hEntry->obs = *itObs;
                hEntry->immediateReward = *itRew;
                hEntry->partOfBelNode = b;
                b->add(hEntry);
                hEntry->hasBeenBackup = false;
            } else {
                s = allStates->add(std::move(*itSt));
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
            std::vector<HistoryEntry *>::iterator itHistEntry;
            for (itHistEntry = history->histSeq.begin() + hIdx;
                    itHistEntry != history->histSeq.begin() + nOrgEntries + 1;
                    itHistEntry++) {
                (*itHistEntry)->stateInfo->delUsedInHistEntry(*itHistEntry);
            }
            history->histSeq.erase(history->histSeq.begin() + hIdx,
                                   history->histSeq.begin() + nOrgEntries + 1);
        }

        long sIdx = hIdx;
        std::vector<HistoryEntry *>::iterator itH;
        for (itH = history->histSeq.begin() + sIdx;
                itH != history->histSeq.end(); itH++, hIdx++) {
            currDisc = currDisc * model->getDiscount();
            (*itH)->discount = currDisc;
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
        std::vector<HistoryEntry *>::iterator itHistEntry;
        for (itHistEntry = history->histSeq.begin() + history->startAffectedIdx;
                itHistEntry != history->histSeq.end(); itHistEntry++) {
            (*itHistEntry)->stateInfo->delUsedInHistEntry(*itHistEntry);
        }
        history->histSeq.erase(
            history->histSeq.begin() + history->startAffectedIdx,
            history->histSeq.end());
    }
    /*
     long hIdx = 0;
     cerr << "Size new hist: " << history->histSeq.size() << endl;
     for (std::vector<HistoryEntry *>::iterator itH = history->histSeq.begin(); itH != history->histSeq.end(); itH++, hIdx++) {
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
    for (long i = endAffectedIdx - 1; i >= startAffectedIdx; i--) {
        currHistEntry = histSeq->histSeq[i];
        prevRew = currHistEntry->qVal;
        currHistEntry->immediateReward = model->getReward(
                                             *currHistEntry->stateInfo->getState(), currHistEntry->actId);
        totRew = currHistEntry->qVal = currHistEntry->discount
                                       * currHistEntry->immediateReward + totRew;
        currHistEntry->partOfBelNode->updateVal(currHistEntry->actId, prevRew,
                                                totRew, false);
    }

    for (long i = startAffectedIdx - 1; i >= 0; i--) {
        currHistEntry = histSeq->histSeq[i];
        prevRew = currHistEntry->qVal;
        totRew = currHistEntry->qVal = currHistEntry->discount
                                       * currHistEntry->immediateReward + totRew;
        currHistEntry->partOfBelNode->updateVal(currHistEntry->actId, prevRew,
                                                totRew, false);
    }
}
