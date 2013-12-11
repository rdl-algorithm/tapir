#include "Solver.hpp"

#include <cfloat>                       // for DBL_MAX
#include <climits>                      // for LONG_MAX
#include <cmath>                        // for pow, exp
#include <ctime>                        // for clock, clock_t, CLOCKS_PER_SEC

#include <algorithm>                    // for max
#include <iostream>                     // for operator<<, cerr, ostream, basic_ostream, endl, basic_ostream<>::__ostream_type, cout
#include <memory>                       // for unique_ptr
#include <random>                       // for bernoulli_distribution, uniform_int_distribution
#include <set>                          // for set, _Rb_tree_const_iterator, set<>::iterator
#include <tuple>                        // for tie, tuple
#include <type_traits>                  // for remove_reference<>::type
#include <utility>                      // for move, make_pair, pair
#include <vector>                       // for vector, vector<>::iterator, vector<>::reverse_iterator

#include "defs.hpp"                     // for make_unique, RandomGenerator

#include "Action.hpp"                   // for Action
#include "BeliefNode.hpp"               // for BeliefNode, BeliefNode::startTime
#include "BeliefTree.hpp"               // for BeliefTree
#include "ChangeType.hpp"               // for ChangeType, ChangeType::UNDEFINED, ChangeType::ADDOBSERVATION, ChangeType::ADDOBSTACLE, ChangeType::ADDSTATE, ChangeType::DELSTATE, ChangeType::REWARD, ChangeType::TRANSITION
#include "Histories.hpp"                // for Histories
#include "HistoryEntry.hpp"             // for HistoryEntry
#include "HistorySequence.hpp"          // for HistorySequence
#include "Model.hpp"                    // for Model::StepResult, Model
#include "Observation.hpp"              // for Observation
#include "State.hpp"                    // for State
#include "StateInfo.hpp"                // for StateInfo
#include "StatePool.hpp"                // for StatePool

using std::cerr;
using std::cout;
using std::endl;

Solver::Solver(RandomGenerator *randGen, std::unique_ptr<Model> model) :
    randGen(randGen),
    model(std::move(model)),
    policy(std::make_unique<BeliefTree>()),
    allHistories(std::make_unique<Histories>()),
    allStates(std::make_unique<StatePool>()),
    lastRolloutMode(ROLLOUT_RANDHEURISTIC),
    exploreCoef(this->model->getExploreCoef()),
    cRollout{ 1.0, 1.0 },
    wRollout{ 1.0, 1.0 },
    pRollout{ 0.5, 0.5 },
    nUsedRollout{ 1, 1 } {
}

// Default destructor, not in .hpp
Solver::~Solver() {
}

void Solver::genPol(long maxTrials, double depthTh) {
    double disc = model->getDiscount();
    BeliefNode *root = policy->getRoot();

    // Set root to have at least one particle for each possible action.
    for (Action action = 0; action < (long)model->getNActions(); action++) {
        std::unique_ptr<State> newState = model->sampleAnInitState();
        StateInfo *stateInfo = allStates->add(std::move(newState));
        State *state = stateInfo->getState();

        std::unique_ptr<HistoryEntry> newHistEntry =
            std::make_unique<HistoryEntry>(stateInfo);
        HistoryEntry *currHistEntry = newHistEntry.get();
        currHistEntry->partOfBelNode = root;
        root->add(currHistEntry);
        stateInfo->addHistoryEntry(currHistEntry);
        stateInfo->addBeliefNode(root);

        std::unique_ptr<HistorySequence> newHistSeq = (
                std::make_unique<HistorySequence>(std::move(newHistEntry), 0));
        HistorySequence *currHistSeq = newHistSeq.get();
        allHistories->add(std::move(newHistSeq));

        // Use the generative model to step forward.
        Model::StepResult result = model->generateStep(*state, action);
        State *nextState = result.nextState.get();
        stateInfo = allStates->add(std::move(result.nextState));

        currHistEntry->discount = 1.0;
        currHistEntry->immediateReward = result.immediateReward;
        currHistEntry = currHistSeq->addEntry(action, result.observation,
                    stateInfo);
        BeliefNode *currNode;
        bool isNew;
        std::tie(currNode, isNew) = root->addChild(action, result.observation);
        currNode->add(currHistEntry);
        if (isNew) {
            policy->allNodes.push_back(currNode);
        }
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

        currHistEntry->qVal = result.immediateReward + currHistEntry->qVal;
        root->updateQValue(action, currHistEntry->qVal);
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
    double initialStartNodeQVal = startNode->getBestMeanQValue();

    BeliefNode *currNode = startNode;
    double currentDiscount = std::pow(discountFactor, startDepth);

    std::unique_ptr<HistoryEntry> newHistEntry = std::make_unique<HistoryEntry>(
                startStateInfo);
    HistoryEntry *currHistEntry = newHistEntry.get();
    currNode->add(currHistEntry);
    currHistEntry->partOfBelNode = currNode;
    currHistEntry->discount = currentDiscount;

    std::unique_ptr<HistorySequence> newHistSeq = (
            std::make_unique<HistorySequence>(std::move(newHistEntry),
                    startDepth));
    HistorySequence *currHistSeq = newHistSeq.get();
    allHistories->add(std::move(newHistSeq));

    bool rolloutUsed = false;
    bool done = false;
    while (!done && currentDiscount > depthTh) {
        Model::StepResult result;
        double qVal = 0;
        if (model->getNActions()
            == (unsigned long)currNode->getNActChildren()) {
            // If all actions have been attempted, use UCB
            long action = currNode->getUCBAction();
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

        bool isNew;
        std::tie(currNode, isNew) = currNode->addChild(result.action,
                    result.observation);
        currNode->add(currHistEntry);
        if (isNew) {
            policy->allNodes.push_back(currNode);
        }
        currHistEntry->partOfBelNode = currNode;

        nextStateInfo->addHistoryEntry(currHistEntry);
        nextStateInfo->addBeliefNode(currNode);

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
        updWeightRolloutAct(
                policy->root->getBestMeanQValue() - initialStartNodeQVal);
    }
    rolloutUsed = false;
}

void Solver::backup(HistorySequence *history) {
    std::vector<std::unique_ptr<HistoryEntry>>::reverse_iterator itHist =
        history->histSeq.rbegin();
    double totRew;
    if ((*itHist)->action == -1) {
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
            (*itHist)->partOfBelNode->updateQValue((*itHist)->action,
                    prevTotRew,
                    totRew, false);
        } else {
            totRew = (*itHist)->qVal = (*itHist)->discount
                    * (*itHist)->immediateReward + totRew;
            (*itHist)->partOfBelNode->updateQValue((*itHist)->action, totRew);
            (*itHist)->hasBeenBackup = true;
        }
    }
}

std::pair<Model::StepResult, double> Solver::getRolloutAction(
        BeliefNode *belNode, State &state, double startDisc, double disc) {
    Action action;
    Model::StepResult result;
    double qVal;

    if (std::bernoulli_distribution(pRollout[ROLLOUT_RANDHEURISTIC])(*randGen))
    {
        lastRolloutMode = ROLLOUT_RANDHEURISTIC;
    } else {
        lastRolloutMode = ROLLOUT_POL;
    }
    std::clock_t startTime, endTime;
    if (lastRolloutMode == ROLLOUT_POL) {
        startTime = std::clock();
        BeliefNode *currNode = getNNBelNode(belNode);
        if (currNode == nullptr) {
            lastRolloutMode = ROLLOUT_RANDHEURISTIC; // Use RANDHEURISTIC instead.
        } else {
            action = belNode->getNextActionToTry();
            result = model->generateStep(state, action);
            currNode = currNode->getChild(action, result.observation);
            qVal = (startDisc
                    * rolloutPolHelper(currNode, *result.nextState, disc));
            lastRolloutMode = ROLLOUT_POL;
            endTime = std::clock();
        }
    }
    if (lastRolloutMode == ROLLOUT_RANDHEURISTIC) {
        startTime = std::clock();
        action = belNode->getNextActionToTry();
        result = model->generateStep(state, action);
        if (result.isTerminal) {
            qVal = model->getReward(*result.nextState);
        } else {
            qVal = model->solveHeuristic(*result.nextState);
        }
        qVal *= startDisc * disc;
        lastRolloutMode = ROLLOUT_RANDHEURISTIC;
        endTime = std::clock();
    }
    cRollout[lastRolloutMode] +=
        (endTime - startTime) * 1000.0 / CLOCKS_PER_SEC;
    nUsedRollout[lastRolloutMode]++;

    return std::make_pair(std::move(result), qVal);
}

double Solver::rolloutPolHelper(BeliefNode *currNode, State &state,
        double disc) {
    if (currNode == nullptr) {
        // cerr << "nullptr in rolloutPolHelper!" << endl;
        return 0.0;
    } else if (currNode->getNParticles() == 0) {
        cerr << "nParticles == 0 in rolloutPolHelper" << endl;
        return 0.0;
    } else if (currNode->getNActChildren() == 0) {
        // cerr << "No children in rolloutPolHelper" << endl;
        return 0.0;
    }

    long action = currNode->getBestAction();
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
    wRollout[lastRolloutMode] = wRollout[lastRolloutMode]
        * std::exp(
                exploreCoef * (valImprovement / model->getMaxVal())
                / (2 * pRollout[lastRolloutMode]));
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
        std::vector<Action> &trajAction, std::vector<Observation> &trajObs,
        std::vector<double> &trajRew, long *actualNSteps, double *totChTime,
        double *totImpTime) {
    trajSt.clear();
    trajAction.clear();
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
            std::vector<std::unique_ptr<State>> affectedRange;
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

        trajAction.push_back(result.action);
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
            nextNode = addChild(currNode, trajAction.back(), trajObs.back(), i);
        }
        currNode = nextNode;
    }
    return discountedTotalReward;
}

Model::StepResult Solver::simAStep(BeliefNode *currentBelief,
        State &currentState) {
    State *state = currentBelief->sampleAParticle(randGen)->getState();
    cerr << "Sampled particle: " << *state << endl;
    double totalDistance = 0;
    for (int i = 0; i < 100; i++) {
        State *s1 = currentBelief->sampleAParticle(randGen)->getState();
        State *s2 = currentBelief->sampleAParticle(randGen)->getState();
        totalDistance += s1->distanceTo(*s2);
    }
    cerr << "Mean sampled distance: " << totalDistance / 100 << endl;

    long action = currentBelief->getBestAction();
    if (action == -1) {
        action = std::uniform_int_distribution<long>(
                    0, model->getNActions() - 1)(*randGen);
    }
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
        + allHistories->getHistorySequence(entry->getSeqId())->startDepth;
    samples.push_back(entry->stateInfo);
    for (long i = 0; i < maxTrials - 1; i++) {
        samples.push_back(startNode->sampleAParticle(randGen)->stateInfo);
    }
    for (StateInfo *sample : samples) {
        singleSearch(startNode, sample, depth, disc, depthTh);
    }
}

BeliefNode *Solver::addChild(BeliefNode *currNode, Action &action,
        Observation &obs,
        long timeStep) {
    cerr << "In add particle due to depletion" << endl;
    BeliefNode *nextNode = nullptr;

    std::vector<State *> particles;
    std::vector<HistoryEntry *>::iterator it;
    for (HistoryEntry *entry : currNode->particles) {
        particles.push_back(entry->stateInfo->getState());
    }

    double disc = model->getDiscount();
    double currentDiscount = std::pow(disc, timeStep);
// Attempt to generate particles for next state based on the current belief,
// the observation, and the action.
    std::vector<std::unique_ptr<State>> nextParticles(
            model->generateParticles(action, obs, particles));
    if (nextParticles.empty()) {
        cerr << "Could not generate based on belief!" << endl;
        // If that fails, ignore the current belief.
        nextParticles = model->generateParticles(action, obs);
    }
    if (nextParticles.empty()) {
        cerr << "Could not generate new particles!" << endl;
    }

    bool isNew;
    std::tie(nextNode, isNew) = currNode->addChild(action, obs);
    if (isNew) {
        policy->allNodes.push_back(nextNode);
    } else {
        cerr << "Child should not have existed..." << endl;
    }

    for (std::unique_ptr<State> &uniqueStatePtr : nextParticles) {

        StateInfo *stateInfo = allStates->add(std::move(uniqueStatePtr));
        State *state = stateInfo->getState();

        std::unique_ptr<HistoryEntry> newHistEntry =
            std::make_unique<HistoryEntry>(stateInfo);
        HistoryEntry *histEntry = newHistEntry.get();
        nextNode->add(histEntry);
        histEntry->partOfBelNode = nextNode;
        stateInfo->addHistoryEntry(histEntry);
        stateInfo->addBeliefNode(nextNode);

        std::unique_ptr<HistorySequence> newHistSeq = (
                std::make_unique<HistorySequence>(std::move(newHistEntry),
                        timeStep));
        HistorySequence *histSeq = newHistSeq.get();
        allHistories->add(std::move(newHistSeq));

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
    return nextNode;
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
        std::vector<std::unique_ptr<State>> &affectedRange,
        std::vector<ChangeType> &chTypes,
        std::set<HistorySequence *> &affectedHistSeq) {
    // Get affected states
    std::set<StateInfo *> affectedStates;
    std::vector<std::unique_ptr<State>>::iterator it1, it2;
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
                histSeq = allHistories->getHistorySequence((*itH)->seqId);

                // Set affected
                if ((*itH)->entryId < histSeq->startAffectedIdx) {
                    histSeq->startAffectedIdx = (*itH)->entryId;
                }
                if ((*itH)->entryId > histSeq->endAffectedIdx) {
                    histSeq->endAffectedIdx = (*itH)->entryId;
                }
                histSeq->changeType = std::max(histSeq->changeType,
                            (*itS)->chType);
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
            currHistEntry = sequence->get(sequence->endAffectedIdx);
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
                            currHistEntry->action);
                double prevRew = currHistEntry->qVal;
                currHistEntry->qVal =
                    (currHistEntry->discount
                     * currHistEntry->immediateReward
                     + sequence->histSeq[sequence->endAffectedIdx
                                         + 1]->qVal);
                currHistEntry->partOfBelNode->updateQValue(
                        currHistEntry->action,
                        prevRew, currHistEntry->qVal, false);
            }
            updateVal(sequence);
            break;
        }
        case ChangeType::ADDOBSTACLE: {
            std::vector<State const *> states = sequence->getStates();
            std::vector<std::unique_ptr<State>> modifStSeq;
            std::vector<Action> modifActSeq;
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
    std::vector<std::unique_ptr<HistoryEntry>>::iterator itHist;
    for (itHist = history->histSeq.begin() + 1;
         itHist != history->histSeq.end(); itHist++) {
        (*itHist)->partOfBelNode->updateQValue((*itHist)->action,
                (*itHist)->immediateReward, 0, true);
        (*itHist)->hasBeenBackup = false;
    }
}

void Solver::modifHistSeqFr(HistorySequence *history,
        std::vector<std::unique_ptr<State>> &modifStSeq,
        std::vector<Action> &modifActSeq,
        std::vector<Observation> &modifObsSeq,
        std::vector<double> &modifRewSeq) {

    std::vector<std::unique_ptr<State>>::iterator itSt = modifStSeq.begin();
    std::vector<Action>::iterator itAct = modifActSeq.begin();
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
       cerr << " act " << (*itH1)->action;
       //cerr << " obs " << (*itH1)->obs[0] << " " << (*itH1)->obs[1];
       cerr << " rew " << (*itH1)->rew << endl;
       }
       }
     */
    long hIdx = history->startAffectedIdx - 1;
    double currDisc;
    if (itAct != modifActSeq.end()) {
        hEntry = history->get(hIdx);
        currDisc = hEntry->discount;
        hEntry->action = *itAct;
        hEntry->obs = *itObs;
        hEntry->immediateReward = *itRew;
        bool isNew;
        std::tie(b, isNew) = hEntry->partOfBelNode->addChild(*itAct, *itObs);
        if (isNew) {
            policy->allNodes.push_back(b);
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
                hEntry = history->get(hIdx);
                s = allStates->add(std::move(*itSt));
                hEntry->stateInfo = s;
                hEntry->action = *itAct;
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
            std::tie(b, isNew) = hEntry->partOfBelNode->addChild(*itAct,
                        *itObs);
            if (isNew) {
                policy->allNodes.push_back(b);
            }
        }
        if (itSt != modifStSeq.end()) {
            currDisc = currDisc * model->getDiscount();
            if (hIdx < nOrgEntries) {
                hEntry = history->get(hIdx);
                s = allStates->add(std::move(*itSt));
                hEntry->stateInfo = s;
                hEntry->action = -1;
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
    std::vector<std::unique_ptr<HistoryEntry>>::iterator itHistEntry;
    for (itHistEntry = history->histSeq.begin() + hIdx;
         itHistEntry != history->histSeq.end(); itHistEntry++) {
        (*itHistEntry)->stateInfo->delUsedInHistEntry(itHistEntry->get());
    }
    history->histSeq.erase(history->histSeq.begin() + hIdx,
            history->histSeq.end());
    /*
       std::vector<HistoryEntry *>::iterator itH;
       hIdx = 0;
       cerr << "Size new hist: " << history->histSeq.size() << endl;
       for (itH = history->histSeq.begin(); itH != history->histSeq.end(); itH++, hIdx++) {
       cerr << "histEntry-" << hIdx << " " << (*itH)->st->s[0] << " " << (*itH)->st->s[1] << " act " << (*itH)->action << endl;
       //" obs " << (*itH)->obs[0] << " " << (*itH)->obs[1] << " rew " << (*itH)->rew << endl;
       }
       cerr << "About to backup\n";
     */
    history->fixEntryIds();
    backup(history);
//cerr << "After backup\n";
}

void Solver::modifHistSeqFrTo(HistorySequence *history,
        std::vector<std::unique_ptr<State>> &modifStSeq,
        std::vector<Action> &modifActSeq, std::vector<Observation> &modifObsSeq,
        std::vector<double> &modifRewSeq) {

    std::vector<std::unique_ptr<State>>::iterator itSt = modifStSeq.begin();
    std::vector<Action>::iterator itAct = modifActSeq.begin();
    std::vector<Observation>::iterator itObs = modifObsSeq.begin();
    std::vector<double>::iterator itRew = modifRewSeq.begin();

    BeliefNode *b = nullptr;
    StateInfo *s;
    HistoryEntry *hEntry;

    long nOrgEntries = history->endAffectedIdx;

    if (itAct != modifActSeq.end()) {
        long hIdx = history->startAffectedIdx - 1;
        hEntry = history->get(hIdx);
        double currDisc = hEntry->discount;
        hEntry->action = *itAct;
        hEntry->obs = *itObs;
        hEntry->immediateReward = *itRew;

        hEntry->hasBeenBackup = false;
        bool isNew;
        std::tie(b, isNew) = hEntry->partOfBelNode->addChild(*itAct, *itObs);
        if (isNew) {
            policy->allNodes.push_back(b);
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
                hEntry = history->get(hIdx);
                s = allStates->add(std::move(*itSt));
                hEntry->stateInfo = s;
                hEntry->action = *itAct;
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
            std::tie(b, isNew) = hEntry->partOfBelNode->addChild(*itAct,
                        *itObs);
            if (isNew) {
                policy->allNodes.push_back(b);
            }
        }

        if (hIdx <= nOrgEntries) {
            std::vector<std::unique_ptr<HistoryEntry>>::iterator itHistEntry;
            for (itHistEntry = history->histSeq.begin() + hIdx;
                 itHistEntry != history->histSeq.begin() + nOrgEntries + 1;
                 itHistEntry++) {
                (*itHistEntry)->stateInfo->delUsedInHistEntry(itHistEntry->get());
            }
            history->histSeq.erase(history->histSeq.begin() + hIdx,
                    history->histSeq.begin() + nOrgEntries + 1);
        }

        long sIdx = hIdx;
        std::vector<std::unique_ptr<HistoryEntry>>::iterator itH;
        for (itH = history->histSeq.begin() + sIdx;
             itH != history->histSeq.end(); itH++, hIdx++) {
            currDisc = currDisc * model->getDiscount();
            (*itH)->discount = currDisc;
            (*itH)->partOfBelNode = b;
            b->add(itH->get());
            (*itH)->hasBeenBackup = false;
            if ((*itH)->action >= 0) {
                bool isNew;
                std::tie(b, isNew) = hEntry->partOfBelNode->addChild(*itAct,
                            *itObs);
                if (isNew) {
                    policy->allNodes.push_back(b);
                }
            }
        }
    } else {
        std::vector<std::unique_ptr<HistoryEntry>>::iterator itHistEntry;
        for (itHistEntry = history->histSeq.begin() + history->startAffectedIdx;
             itHistEntry != history->histSeq.end(); itHistEntry++) {
            (*itHistEntry)->stateInfo->delUsedInHistEntry(itHistEntry->get());
        }
        history->histSeq.erase(
                history->histSeq.begin() + history->startAffectedIdx,
                history->histSeq.end());
    }
    /*
       long hIdx = 0;
       cerr << "Size new hist: " << history->histSeq.size() << endl;
       for (std::vector<HistoryEntry *>::iterator itH = history->histSeq.begin(); itH != history->histSeq.end(); itH++, hIdx++) {
       cerr << "histEntry-" << hIdx << " " << (*itH)->st->s[0] << " " << (*itH)->st->s[1] << " act " << (*itH)->action;
       //" obs " << (*itH)->obs[0] << " " << (*itH)->obs[1]
       cerr << " rew " << (*itH)->rew << endl;
       }
       cerr << "About to backup\n";
     */

    history->fixEntryIds();
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
        currHistEntry = histSeq->get(i);
        prevRew = currHistEntry->qVal;
        currHistEntry->immediateReward = model->getReward(
                    *currHistEntry->stateInfo->getState(),
                    currHistEntry->action);
        totRew = currHistEntry->qVal = currHistEntry->discount
                * currHistEntry->immediateReward + totRew;
        currHistEntry->partOfBelNode->updateQValue(currHistEntry->action,
                prevRew,
                totRew, false);
    }

    for (long i = startAffectedIdx - 1; i >= 0; i--) {
        currHistEntry = histSeq->get(i);
        prevRew = currHistEntry->qVal;
        totRew = currHistEntry->qVal = currHistEntry->discount
                * currHistEntry->immediateReward + totRew;
        currHistEntry->partOfBelNode->updateQValue(currHistEntry->action,
                prevRew,
                totRew, false);
    }
}
