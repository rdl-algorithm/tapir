#include "Solver.hpp"

#include <cfloat>                       // for DBL_MAX
#include <climits>                      // for LONG_MAX
#include <cmath>                        // for pow, exp
#include <ctime>                        // for clock, clock_t, CLOCKS_PER_SEC

#include <algorithm>                    // for max
#include <iostream>                     // for operator<<, cerr, ostream, basic_ostream, endl, basic_ostream<>::__ostream_type, cout
#include <memory>                       // for unique_ptr
#include <random>                       // for uniform_int_distribution, bernoulli_distribution
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
#include "Serializer.hpp"               // for Serializer
#include "State.hpp"                    // for State, operator<<
#include "StateInfo.hpp"                // for StateInfo
#include "StatePool.hpp"                // for StatePool

#include "RTree.hpp"
#include "SpatialIndexVisitor.hpp"

using std::cerr;
using std::cout;
using std::endl;

namespace solver {
Solver::Solver(RandomGenerator *randGen, std::unique_ptr<Model> model) :
    serializer_(nullptr),
    randGen_(randGen),
    model_(std::move(model)),
    policy_(std::make_unique<BeliefTree>()),
    allHistories_(std::make_unique<Histories>()),
    allStates_(std::make_unique<StatePool>(model_->getNStVars())),
    lastRolloutMode_(ROLLOUT_RANDHEURISTIC),
    heuristicExploreCoefficient_(this->model_->getHeuristicExploreCoefficient()),
    timeUsedPerHeuristic_{1.0, 1.0},
    heuristicWeight_{ 1.0, 1.0 },
    heuristicProbability_{ 0.5, 0.5 },
    heuristicUseCount_{ 1, 1 } {
}

// Default destructor, not in .hpp
Solver::~Solver() {
}

void Solver::setSerializer(Serializer *serializer) {
    serializer_ = serializer;
}

void Solver::registerParticle(BeliefNode *node, HistoryEntry *entry,
        StateInfo *stateInfo) {
    node->addParticle(entry);
    entry->owningBeliefNode_ = node;
    stateInfo->addBeliefNode(node);
    stateInfo->addHistoryEntry(entry);
}

void Solver::deregisterParticle(BeliefNode *node, HistoryEntry *entry,
        StateInfo *stateInfo) {
    node->removeParticle(entry);
    entry->owningBeliefNode_ = nullptr;
    stateInfo->removeBeliefNode(node);
    stateInfo->removeHistoryEntry(entry);
}

void Solver::genPol(unsigned long maxTrials, double minimumDiscount) {
    double discountFactor = model_->getDiscountFactor();
    BeliefNode *root = policy_->getRoot();

    // Initialise the tree by taking each action once.
    for (Action action = 0; action < (long)model_->getNActions(); action++) {
        std::unique_ptr<State> newState = model_->sampleAnInitState();
        StateInfo *stateInfo = allStates_->add(std::move(newState));
        State *state = stateInfo->getState();

        std::unique_ptr<HistoryEntry> newHistEntry =
            std::make_unique<HistoryEntry>(stateInfo);
        HistoryEntry *firstHistEntry = newHistEntry.get();

        registerParticle(root, firstHistEntry, stateInfo);

        std::unique_ptr<HistorySequence> newHistSeq = (
                std::make_unique<HistorySequence>(std::move(newHistEntry), 0));
        HistorySequence *currHistSeq = newHistSeq.get();
        allHistories_->add(std::move(newHistSeq));

        // Use the generative model to step forward.
        Model::StepResult result = model_->generateStep(*state, action);
        firstHistEntry->discount_ = 1.0;
        firstHistEntry->immediateReward_ = result.immediateReward;
        firstHistEntry->action_ = result.action;
        firstHistEntry->observation_ = result.observation;

        // Add the next state to the pool, and retrieve the next state.
        stateInfo = allStates_->add(std::move(result.nextState));
        State *nextState = stateInfo->getState();

        // Step forward in the history sequence, and update the belief node.
        HistoryEntry *currHistEntry = currHistSeq->addEntry(stateInfo);
        BeliefNode *currNode = (
                policy_->addBeliefNode(root, action, result.observation));
        registerParticle(currNode, currHistEntry, stateInfo);

        // We're not going any deeper, so we retrieve the immediate reward for
        // the state alone.
        currHistEntry->discount_ = discountFactor;
        currHistEntry->immediateReward_ = model_->getReward(*nextState);
        currHistEntry->totalDiscountedReward_ = discountFactor * currHistEntry->immediateReward_;
        firstHistEntry->totalDiscountedReward_ = result.immediateReward + currHistEntry->totalDiscountedReward_;
        root->updateQValue(result.action, firstHistEntry->totalDiscountedReward_);
    }

    // Start expanding the tree.
    for (unsigned long i = 0; i < maxTrials; i++) {
        singleSearch(discountFactor, minimumDiscount);
    }
}

void Solver::singleSearch(double discountFactor, double minimumDiscount) {
    StateInfo *stateInfo = allStates_->add(model_->sampleAnInitState());
    singleSearch(
            policy_->getRoot(), stateInfo, 0, discountFactor, minimumDiscount);
}

void Solver::singleSearch(BeliefNode *startNode, StateInfo *startStateInfo,
        long startDepth, double discountFactor, double minimumDiscount) {
    double initialStartNodeQVal = startNode->getBestMeanQValue();

    BeliefNode *currNode = startNode;
    double currentDiscount = std::pow(discountFactor, startDepth);

    std::unique_ptr<HistoryEntry> newHistEntry = std::make_unique<HistoryEntry>(
                startStateInfo);
    HistoryEntry *currHistEntry = newHistEntry.get();
    registerParticle(currNode, currHistEntry, startStateInfo);
    currHistEntry->discount_ = currentDiscount;

    std::unique_ptr<HistorySequence> newHistSeq = (
            std::make_unique<HistorySequence>(std::move(newHistEntry),
                    startDepth));
    HistorySequence *currHistSeq = newHistSeq.get();
    allHistories_->add(std::move(newHistSeq));

    bool rolloutUsed = false;
    bool done = false;
    while (!done && currentDiscount >= minimumDiscount) {
        Model::StepResult result;
        double qVal = 0;
        if (model_->getNActions() == currNode->getNActChildren()) {
            // If all actions have been attempted, use UCB
            long action = currNode->getUcbAction(
                        model_->getUcbExploreCoefficient());
            result = model_->generateStep(
                        *currHistEntry->stateInfo_->getState(),
                        action);
            done = result.isTerminal;
        } else {
            // Otherwise use the rollout method
            std::tie(result, qVal) = getRolloutAction(currNode,
                        *currHistEntry->stateInfo_->getState(),
                        currentDiscount, discountFactor);
            rolloutUsed = true;
            done = true;
        }
        currentDiscount *= discountFactor;
        currHistEntry->discount_ = currentDiscount;
        currHistEntry->immediateReward_ = result.immediateReward;
        currHistEntry->action_ = result.action;
        currHistEntry->observation_ = result.observation;

        // Add the next state to the pool
        StateInfo *nextStateInfo = allStates_->add(std::move(result.nextState));

        // Step forward in the history, and update the belief node.
        currHistEntry = currHistSeq->addEntry(nextStateInfo);
        currNode = policy_->addBeliefNode(currNode, result.action, result.observation);
        registerParticle(currNode, currHistEntry, nextStateInfo);

        if (rolloutUsed) {
            currHistEntry->totalDiscountedReward_ = qVal;
        } else {
            if (done) {
                currHistEntry->immediateReward_ = model_->getReward(
                            *nextStateInfo->getState());
                currHistEntry->totalDiscountedReward_ = currHistEntry->discount_
                    * currHistEntry->immediateReward_;
            }
        }
    }
    backup(currHistSeq);
    if (rolloutUsed) {
        updateHeuristicProbabilities(
                policy_->getRoot()->getBestMeanQValue() - initialStartNodeQVal);
    }
    rolloutUsed = false;
}

void Solver::backup(HistorySequence *history) {
    std::vector<std::unique_ptr<HistoryEntry>>::reverse_iterator itHist = (
            history->histSeq_.rbegin());
    double totRew;
    if ((*itHist)->action_ == -1) {
        totRew = (*itHist)->totalDiscountedReward_;
    } else {
        totRew = (*itHist)->totalDiscountedReward_ = (*itHist)->discount_
                * (*itHist)->immediateReward_;
    }
    itHist++;
    for (; itHist != history->histSeq_.rend(); itHist++) {
        if ((*itHist)->hasBeenBackedUp_) {
            double prevTotRew = (*itHist)->totalDiscountedReward_;
            totRew = (*itHist)->totalDiscountedReward_ = (*itHist)->discount_
                    * (*itHist)->immediateReward_ + totRew;
            (*itHist)->owningBeliefNode_->updateQValue((*itHist)->action_,
                    prevTotRew,
                    totRew, false);
        } else {
            totRew = (*itHist)->totalDiscountedReward_ = (*itHist)->discount_
                    * (*itHist)->immediateReward_ + totRew;
            (*itHist)->owningBeliefNode_->updateQValue((*itHist)->action_,
                    totRew);
            (*itHist)->hasBeenBackedUp_ = true;
        }
    }
}

std::pair<Model::StepResult, double> Solver::getRolloutAction(
        BeliefNode *belNode, State &state, double startDiscount,
        double discountFactor) {
    // We will try the next action that has not yet been tried.
    Action action = belNode->getNextActionToTry();
    Model::StepResult result;
    double qVal;

    if (std::bernoulli_distribution(
                heuristicProbability_[ROLLOUT_RANDHEURISTIC])(*randGen_)) {
        lastRolloutMode_ = ROLLOUT_RANDHEURISTIC;
    } else {
        lastRolloutMode_ = ROLLOUT_POL;
    }
    std::clock_t startTime, endTime;
    if (lastRolloutMode_ == ROLLOUT_POL) {
        startTime = std::clock();
        // Find a nearest neighbor as an approximation.
        BeliefNode *currNode = getNNBelNode(belNode);
        if (currNode == nullptr) {
            lastRolloutMode_ = ROLLOUT_RANDHEURISTIC;
            // Use RANDHEURISTIC instead.
        } else {
            result = model_->generateStep(state, action);
            currNode = currNode->getChild(action, result.observation);
            qVal = startDiscount * rolloutPolHelper(currNode, *result.nextState,
                        discountFactor);
            lastRolloutMode_ = ROLLOUT_POL;
            endTime = std::clock();
        }
    }
    if (lastRolloutMode_ == ROLLOUT_RANDHEURISTIC) {
        startTime = std::clock();
        result = model_->generateStep(state, action);
        if (result.isTerminal) {
            qVal = model_->getReward(*result.nextState);
        } else {
            qVal = model_->solveHeuristic(*result.nextState);
        }
        qVal *= startDiscount * discountFactor;
        lastRolloutMode_ = ROLLOUT_RANDHEURISTIC;
        endTime = std::clock();
    }
    timeUsedPerHeuristic_[lastRolloutMode_] +=
        (endTime - startTime) * 1000.0 / CLOCKS_PER_SEC;
    heuristicUseCount_[lastRolloutMode_]++;

    return std::make_pair(std::move(result), qVal);
}

double Solver::rolloutPolHelper(BeliefNode *currNode, State &state,
        double discountFactor) {
    if (currNode == nullptr) {
        // cerr << "nullptr in rolloutPolHelper!" << endl;
        return 0.0;
    } else if (currNode->getNParticles() == 0) {
        // cerr << "nParticles == 0 in rolloutPolHelper" << endl;
        return 0.0;
    } else if (currNode->getNActChildren() == 0) {
        // cerr << "No children in rolloutPolHelper" << endl;
        return 0.0;
    }

    long action = currNode->getBestAction();
    Model::StepResult result = model_->generateStep(state, action);
    currNode = currNode->getChild(action, result.observation);
    double qVal = result.immediateReward;
    if (result.isTerminal) {
        qVal += discountFactor * model_->getReward(*result.nextState);
    } else {
        qVal += (discountFactor * rolloutPolHelper(
                         currNode, *result.nextState, discountFactor));
    }
    return qVal;
}

BeliefNode *Solver::getNNBelNode(BeliefNode *b) {
    double d, minDist;
    minDist = DBL_MAX;
    BeliefNode *nnBel = b->nnBel_;
    long numTried = 0;
    for (BeliefNode *node : policy_->allNodes_) {
        if (numTried >= model_->getMaxNnComparisons()) {
            break;
        } else {
            if (b->tNNComp_ < node->tLastAddedParticle_) {
                d = b->distL1Independent(node);
                if (d < minDist) {
                    minDist = d;
                    nnBel = node;
                }
            }
            numTried++;
        }
    }
    b->tNNComp_ = (double) (std::clock() - BeliefNode::startTime)
        * 1000/ CLOCKS_PER_SEC;
    b->nnBel_ = nnBel;
    if (minDist > model_->getMaxNnDistance()) {
        return nullptr;
    }
    return nnBel;
}

void Solver::updateHeuristicProbabilities(double valImprovement) {
    if (valImprovement < 0.0) {
        valImprovement = 0.0;
    }
    heuristicWeight_[lastRolloutMode_] *= std::exp(
                heuristicExploreCoefficient_
                * (valImprovement / model_->getMaxVal())
                / (2 * heuristicProbability_[lastRolloutMode_]));
    double totWRollout = 0.0;
    for (int i = 0; i < 2; i++) {
        totWRollout += heuristicWeight_[i];
    }
    double totP = 0.0;
    for (int i = 0; i < 2; i++) {
        heuristicProbability_[i] =
            ((1 - heuristicExploreCoefficient_) * heuristicWeight_[i]
             / totWRollout + heuristicExploreCoefficient_
             / 2) * heuristicUseCount_[i]
            / timeUsedPerHeuristic_[i];
        totP += heuristicProbability_[i];
    }
    for (int i = 0; i < 2; i++) {
        heuristicProbability_[i] /= totP;
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
    unsigned long maxTrials = model_->getMaxTrials();
    double minimumDiscount = model_->getMinimumDiscount();
    double discFactor = model_->getDiscountFactor();
    double currDiscFactor = 1.0;
    double discountedTotalReward = 0.0;

    BeliefNode *currNode = policy_->getRoot();
    std::unique_ptr<State> state(model_->sampleAnInitState());
    State *currentState = state.get();
    trajSt.push_back(std::move(state));

    cerr << "Initial State:" << endl;
    model_->drawState(*currentState, cerr);

    bool changesDone = changeTimes.empty();
    std::vector<long>::iterator itCh = changeTimes.begin();
    for (long i = 0; i < nSteps; i++) {
        cerr << "t-" << i << endl;
        if (!changesDone && i == *itCh) {  // Model changes.
            cerr << "ModelChanges" << endl;

            chTimeStart = std::clock();
#warning Handling changes still needs to be reimplemented.
            std::set<HistorySequence *> affectedHistSeq;
            std::vector<std::unique_ptr<State>> affectedRange;
            std::vector<ChangeType> typeOfChanges;
            model_->update(*itCh, &affectedRange, &typeOfChanges); // Add typeOfChanges
            //identifyAffectedPol(affectedRange, typeOfChanges, affectedHistSeq);
            //updatePol(affectedHistSeq);
            //resetAffected(affectedHistSeq);

            chTimeEnd = std::clock();
            *totChTime = *totChTime
                + ((chTimeEnd - chTimeStart) * 1000 / CLOCKS_PER_SEC);

            itCh++;
            if (itCh == changeTimes.end()) {
                changesDone = true;
            }
        }
        impSolTimeStart = std::clock();
        improveSol(currNode, maxTrials, minimumDiscount);
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
//    cerr << "Belief node: ";
//    serializer_->save(*currentBelief, cerr);

//    struct MyVisitor : public SpatialIndexVisitor {
//        MyVisitor(StatePool *statePool) :
//                    SpatialIndexVisitor(statePool),
//                    states() {
//        }
//        std::vector<StateInfo *> states;
//        void visit(StateInfo *info) {
//            states.push_back(info);
//        }
//    };
//    MyVisitor visitor(allStates_.get());
//    RTree *tree = static_cast<RTree *>(allStates_->getStateIndex());
//    if (model_->getName() == "Tag") {
//        clock_t startTime = std::clock();
//        for (int i = 0; i < 1000; i++) {
//            visitor.states.clear();
//            tree->boxQuery(visitor,
//                    std::vector<double> { 4,  0,  0,  0,  0,},
//                    std::vector<double> { 4,  0,  4,  9,  1,});
//        }
//        clock_t ticks = std::clock() - startTime;
//
//        cerr << "Query results: " << endl;
//        for (StateInfo *info : visitor.states) {
//            cerr << *info->getState() << endl;
//        }
//        cerr << visitor.states.size() << " states; 1000 reps in " << (double)ticks / CLOCKS_PER_SEC << " seconds." << endl;
//    }

    State *state = currentBelief->sampleAParticle(randGen_)->getState();
    cerr << "Sampled particle: " << *state << endl;

    double totalDistance = 0;
    for (int i = 0; i < 100; i++) {
        State *s1 = currentBelief->sampleAParticle(randGen_)->getState();
        State *s2 = currentBelief->sampleAParticle(randGen_)->getState();
        totalDistance += s1->distanceTo(*s2);
    }
    cerr << "Est. mean inter-particle distance: " << totalDistance / 100
         << endl;

    Action action = currentBelief->getBestAction();
    if (action == -1) {
        action = std::uniform_int_distribution<unsigned long>(
                    0, model_->getNActions() - 1)(*randGen_);
    }
    Model::StepResult result = model_->generateStep(currentState, action);
    if (result.isTerminal) {
        cerr << " Reach terminal" << endl;
        result.immediateReward += model_->getDiscountFactor()
            * model_->getReward(*result.nextState);
    }
    cerr << "Action: ";
    model_->dispAct(action, cerr);
    cerr << "; Reward: " << result.immediateReward << "; Obs: ";
    model_->dispObs(result.observation, cerr);
    cerr << endl;
    model_->drawState(*result.nextState, cerr);
    return result;
}

void Solver::improveSol(BeliefNode *startNode, unsigned long maxTrials,
        double minimumDiscount) {
    double disc = model_->getDiscountFactor();
    std::vector<StateInfo *> samples;
    HistoryEntry *entry = startNode->sampleAParticle(randGen_);
    long depth = entry->entryId_
        + allHistories_->getHistorySequence(entry->seqId_)->startDepth_;
    samples.push_back(entry->stateInfo_);
    for (unsigned long i = 0; i < maxTrials - 1; i++) {
        samples.push_back(startNode->sampleAParticle(randGen_)->stateInfo_);
    }
    for (StateInfo *sample : samples) {
        singleSearch(startNode, sample, depth, disc, minimumDiscount);
    }
}

BeliefNode *Solver::addChild(BeliefNode *currNode, Action &action,
        Observation &obs,
        long timeStep) {
    cerr << "In add particle due to depletion" << endl;

    std::vector<State *> particles;
    std::vector<HistoryEntry *>::iterator it;
    for (HistoryEntry *entry : currNode->particles_) {
        particles.push_back(entry->stateInfo_->getState());
    }

    double disc = model_->getDiscountFactor();
    double currentDiscount = std::pow(disc, timeStep);
    // Attempt to generate particles for next state based on the current belief,
    // the observation, and the action.
    std::vector<std::unique_ptr<State>> nextParticles(
            model_->generateParticles(action, obs, particles));
    if (nextParticles.empty()) {
        cerr << "Could not generate based on belief!" << endl;
        // If that fails, ignore the current belief.
        nextParticles = model_->generateParticles(action, obs);
    }
    if (nextParticles.empty()) {
        cerr << "Could not generate new particles!" << endl;
    }

    BeliefNode *nextNode = policy_->addBeliefNode(currNode, action, obs);
    for (std::unique_ptr<State> &uniqueStatePtr : nextParticles) {
        StateInfo *stateInfo = allStates_->add(std::move(uniqueStatePtr));
        State *state = stateInfo->getState();

        std::unique_ptr<HistoryEntry> newHistEntry =
            std::make_unique<HistoryEntry>(stateInfo);
        HistoryEntry *histEntry = newHistEntry.get();
        registerParticle(nextNode, histEntry, stateInfo);

        std::unique_ptr<HistorySequence> newHistSeq = (
                std::make_unique<HistorySequence>(std::move(newHistEntry),
                        timeStep));
        HistorySequence *histSeq = newHistSeq.get();
        allHistories_->add(std::move(newHistSeq));

        // Assign value to the new history entry
        histEntry->immediateReward_ = model_->getReward(*state);
        histEntry->discount_ = currentDiscount;
        if (model_->isTerm(*state)) {
            histEntry->totalDiscountedReward_ = currentDiscount * histEntry->immediateReward_;
        } else {
            histEntry->totalDiscountedReward_ = currentDiscount
                * (histEntry->immediateReward_
                   + disc * model_->getDefaultVal());
        }
        backup(histSeq);
    }
    return nextNode;
}
} /* namespace solver */
