#include "Solver.hpp"

#include <cmath>                        // for pow, exp
#include <ctime>                        // for clock, clock_t, CLOCKS_PER_SEC

#include <algorithm>                    // for max
#include <iostream>                     // for operator<<, ostream, basic_ostream, endl, basic_ostream<>::__ostream_type, cout
#include <limits>
#include <memory>                       // for unique_ptr
#include <random>                       // for uniform_int_distribution, bernoulli_distribution
#include <set>                          // for set, _Rb_tree_const_iterator, set<>::iterator
#include <tuple>                        // for tie, tuple
#include <type_traits>                  // for remove_reference<>::type
#include <utility>                      // for move, make_pair, pair
#include <vector>                       // for vector, vector<>::iterator, vector<>::reverse_iterator

#include "global.hpp"                     // for RandomGenerator

#include "abstract-problem/Action.hpp"                   // for Action
#include "abstract-problem/Model.hpp"                    // for Model::StepResult, Model
#include "abstract-problem/Observation.hpp"              // for Observation
#include "abstract-problem/State.hpp"                    // for State, operator<<

#include "changes/ChangeFlags.hpp"               // for ChangeFlags, ChangeFlags::UNCHANGED, ChangeFlags::ADDOBSERVATION, ChangeFlags::ADDOBSTACLE, ChangeFlags::ADDSTATE, ChangeFlags::DELSTATE, ChangeFlags::REWARD, ChangeFlags::TRANSITION

#include "mappings/ActionMapping.hpp"
#include "mappings/ObservationMapping.hpp"

#include "serialization/Serializer.hpp"               // for Serializer

#include "indexing/RTree.hpp"
#include "indexing/SpatialIndexVisitor.hpp"

#include "ActionNode.hpp"               // for BeliefNode, BeliefNode::startTime
#include "BeliefNode.hpp"               // for BeliefNode, BeliefNode::startTime
#include "BeliefTree.hpp"               // for BeliefTree
#include "Histories.hpp"                // for Histories
#include "HistoryEntry.hpp"             // for HistoryEntry
#include "HistorySequence.hpp"          // for HistorySequence
#include "StateInfo.hpp"                // for StateInfo
#include "StatePool.hpp"                // for StatePool

using std::cout;
using std::endl;

namespace solver {
Solver::Solver(RandomGenerator *randGen, std::unique_ptr<Model> model) :
    serializer_(nullptr),
    randGen_(randGen),
    model_(std::move(model)),
    actionPool_(model_->createActionPool()),
    observationPool_(model_->createObservationPool()),
    allStates_(std::make_unique<StatePool>(model_->createStateIndex())),
    allHistories_(std::make_unique<Histories>()),
    policy_(std::make_unique<BeliefTree>()),
    historyCorrector_(model_->createHistoryCorrector()),
    lastRolloutMode_(ROLLOUT_RANDHEURISTIC),
    heuristicExploreCoefficient_(this->model_->getHeuristicExploreCoefficient()),
    timeUsedPerHeuristic_{ 1.0, 1.0 },
    heuristicWeight_{ 1.0, 1.0 },
    heuristicProbability_{ 0.5, 0.5 },
    heuristicUseCount_{ 1, 1 } {
}

// Default destructor, not in .hpp
Solver::~Solver() {
}

void Solver::initialize() {
    actionPool_->observationPool_ = observationPool_.get();
    observationPool_->actionPool_ = actionPool_.get();
    policy_->setRoot(std::make_unique<BeliefNode>(
            actionPool_->createActionMapping(), 0));
    historyCorrector_->setSolver(this);
}

void Solver::setSerializer(Serializer *serializer) {
    serializer_ = serializer;
}

void Solver::genPol(long historiesPerStep, long maximumDepth) {
    // Start expanding the tree.
    for (long i = 0; i < historiesPerStep; i++) {
        singleSearch(model_->getDiscountFactor(), maximumDepth);
    }
    cout << "MDP heuristic used ";
    cout << heuristicUseCount_[ROLLOUT_RANDHEURISTIC] - 1 << " times; Took ";
    cout << timeUsedPerHeuristic_[ROLLOUT_RANDHEURISTIC] << "ms" << endl;
    cout << "NN heuristic used ";
    cout << heuristicUseCount_[ROLLOUT_POL] - 1 << " times; Took ";
    cout << timeUsedPerHeuristic_[ROLLOUT_POL] << "ms" << endl;
}

void Solver::singleSearch(double discountFactor, long maximumDepth) {
    StateInfo *stateInfo = allStates_->createOrGetInfo(*model_->sampleAnInitState());
    singleSearch(policy_->getRoot(), stateInfo, 0, discountFactor, maximumDepth);
}

void Solver::singleSearch(BeliefNode *startNode, StateInfo *startStateInfo,
        long startDepth, double discountFactor, long maximumDepth) {
    HistorySequence *sequence = allHistories_->addNew(startDepth);
    HistoryEntry *entry = sequence->addEntry(startStateInfo,
            std::pow(discountFactor, startDepth));
    entry->registerNode(startNode);
    continueSearch(sequence, discountFactor, maximumDepth);
}

void Solver::continueSearch(HistorySequence *sequence,
        double discountFactor, long maximumDepth) {
    while (true) {
        HistoryEntry *lastEntry = sequence->getEntry(sequence->getLength() - 1);
        if (model_->isTerminal(*lastEntry->getState())) {
            debug::show_message("WARNING: Attempted to continue sequence from"
                    " a terminal state.");
        }
    }
    HistorySequence *currHistSeq = sequence;
    HistoryEntry *currHistEntry = sequence->getEntry(
            sequence->histSeq_.size() - 1);
    double currentDiscount = currHistEntry->discount_;
    BeliefNode *currNode = currHistEntry->owningBeliefNode_;

    BeliefNode *sequenceRoot = sequence->getEntry(0)->owningBeliefNode_;
    double initialRootQValue = sequenceRoot->getBestMeanQValue();

    bool rolloutUsed = false;
    bool done = model_->isTerminal(*currHistEntry->getState());
    if (done) {
        debug::show_message("WARNING: Attempted to continue sequence from"
                " a terminal state.");
    }

    long currentDepth = currHistSeq->startDepth_ + currHistEntry->entryId_ + 1;
    while (!done && currentDepth <= maximumDepth) {
        currentDepth++;
        Model::StepResult result;
        double qVal = 0;
        if (!currNode->hasActionToTry()) {
            // If all actions have been attempted, use UCB
            std::unique_ptr<Action> action = currNode->getSearchAction();
            result = model_->generateStep(*currHistEntry->getState(), *action);
            done = result.isTerminal;
        } else {
            // Otherwise use the rollout method
            std::tie(result, qVal) = getRolloutAction(currNode,
                        *currHistEntry->getState(), currentDiscount,
                        discountFactor);
            rolloutUsed = true;
            done = true;
        }
        sequence->isTerminal_ = result.isTerminal;
        currHistEntry->reward_ = result.reward;
        currHistEntry->action_ = result.action->copy();
        currHistEntry->transitionParameters_ = std::move(
                result.transitionParameters);
        currHistEntry->observation_ = result.observation->copy();

        // Add the next state to the pool
        StateInfo *nextStateInfo = allStates_->createOrGetInfo(*result.nextState);

        // Step forward in the history, and update the belief node.
        currentDiscount *= discountFactor;
        currHistEntry = currHistSeq->addEntry(nextStateInfo, currentDiscount);
        currNode = policy_->createOrGetChild(currNode, *result.action,
                *result.observation);
        currHistEntry->registerNode(currNode);

        if (rolloutUsed) {
            // Assign a heuristic value to the final history entry.
            currHistEntry->totalDiscountedReward_ = qVal;
        }
    }
    backup(currHistSeq);
    if (rolloutUsed) {
        updateHeuristicProbabilities(
                sequenceRoot->getBestMeanQValue() - initialRootQValue);
    }
    rolloutUsed = false;
}

void Solver::backup(HistorySequence *sequence) {
    std::vector<std::unique_ptr<HistoryEntry>>::reverse_iterator itHist = (
            sequence->histSeq_.rbegin());
    double totalReward = (*itHist)->totalDiscountedReward_;
    (*itHist)->hasBeenBackedUp_ = true;
    if ((*itHist)->action_ != nullptr) {
        debug::show_message("ERROR: End of sequence has an action!?");
    }
    itHist++;
    bool isFirst = true;
    bool propagating = true;
    double deltaQValue = 0;
    for (; itHist != sequence->histSeq_.rend(); itHist++) {
        HistoryEntry *entry = itHist->get();
        BeliefNode *node = entry->owningBeliefNode_;
        double previousTotalReward;
        if (entry->hasBeenBackedUp_) {
            previousTotalReward = entry->totalDiscountedReward_;
        } else {
            previousTotalReward = 0;
        }
        totalReward = entry->totalDiscountedReward_ = entry->discount_
                * entry->reward_ + totalReward;
        if (propagating) {
            if (isFirst) {
                deltaQValue = totalReward - previousTotalReward;
            } else {
                deltaQValue *= model_->getDiscountFactor();
            }
            double previousQValue = node->getBestMeanQValue();
            node->updateQValue(*entry->action_, deltaQValue,
                    entry->hasBeenBackedUp_ ? 0 : +1);
            deltaQValue = node->getBestMeanQValue() - previousQValue;
            if (deltaQValue == 0) {
                propagating = false;
            }
        }
        entry->hasBeenBackedUp_ = true;
    }
}

void Solver::undoBackup(HistorySequence *sequence) {
    std::vector<std::unique_ptr<HistoryEntry>>::reverse_iterator itHist =
            (sequence->histSeq_.rbegin());
    if (!(*itHist)->hasBeenBackedUp_) {
        debug::show_message("ERROR: Trying to undo but not backed up!?");
    }
    (*itHist)->hasBeenBackedUp_ = false;
    itHist++;
    bool isFirst = true;
    bool propagating = true;
    double deltaQValue = 0;
    for (; itHist != sequence->histSeq_.rend(); itHist++) {
        HistoryEntry *entry = itHist->get();
        BeliefNode *node = entry->owningBeliefNode_;
        if (!entry->hasBeenBackedUp_) {
            debug::show_message("ERROR: Trying to undo but not backed up!?");
        }
        if (propagating) {
            if (isFirst) {
                deltaQValue = -entry->totalDiscountedReward_;
            } else {
                deltaQValue *= model_->getDiscountFactor();
            }
            double previousQValue = node->getBestMeanQValue();
            node->updateQValue(*entry->action_, deltaQValue,
                    entry->hasBeenBackedUp_ ? 0 : -1);
            deltaQValue = node->getBestMeanQValue() - previousQValue;
            if (deltaQValue == 0) {
                propagating = false;
            }
        }
        entry->hasBeenBackedUp_ = false;
    }
}

std::pair<Model::StepResult, double> Solver::getRolloutAction(
        BeliefNode *belNode, State const &state, double startDiscount,
        double discountFactor) {
    // We will try the next action that has not yet been tried.
    std::unique_ptr<Action> action = belNode->getRolloutActions();
    Model::StepResult result = model_->generateStep(state, *action);
    double qVal = 0;

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
            timeUsedPerHeuristic_[ROLLOUT_POL] +=
                (std::clock() - startTime) * 1000.0 / CLOCKS_PER_SEC;
            lastRolloutMode_ = ROLLOUT_RANDHEURISTIC;
            // Use RANDHEURISTIC instead.
        } else {
            currNode = currNode->getChild(*action, *result.observation);
            qVal = rolloutPolHelper(currNode, *result.nextState,
                        discountFactor);
            qVal *= startDiscount * discountFactor;
            lastRolloutMode_ = ROLLOUT_POL;
            endTime = std::clock();
        }
    }
    if (lastRolloutMode_ == ROLLOUT_RANDHEURISTIC) {
        startTime = std::clock();
        if (!result.isTerminal) {
            qVal = model_->getHeuristicValue(*result.nextState);
            qVal *= startDiscount * discountFactor;
        }
        lastRolloutMode_ = ROLLOUT_RANDHEURISTIC;
        endTime = std::clock();
    }
    timeUsedPerHeuristic_[lastRolloutMode_] +=
        (endTime - startTime) * 1000.0 / CLOCKS_PER_SEC;
    heuristicUseCount_[lastRolloutMode_]++;

    return std::make_pair(std::move(result), qVal);
}

double Solver::rolloutPolHelper(BeliefNode *currNode, State const &state,
        double discountFactor) {
    if (currNode == nullptr) {
        // debug::show_message("WARNING: nullptr in rolloutPolHelper!");
        return model_->getHeuristicValue(state);
    } else if (currNode->getNParticles() == 0) {
        // debug::show_message("WARNING: nParticles == 0 in rolloutPolHelper");
        return model_->getHeuristicValue(state);
    } else if (currNode->getNActChildren() == 0) {
        // debug::show_message("WARNING: No children in rolloutPolHelper");
        return model_->getHeuristicValue(state);
    }

    std::unique_ptr<Action> action = currNode->getBestAction();
    Model::StepResult result = model_->generateStep(state, *action);
    currNode = currNode->getChild(*action, *result.observation);
    double qVal = result.reward;
    if (!result.isTerminal) {
        qVal += (discountFactor * rolloutPolHelper(
                         currNode, *result.nextState, discountFactor));
    }
    return qVal;
}

BeliefNode *Solver::getNNBelNode(BeliefNode *currentBelief) {
    if (model_->getMaxNnDistance() < 0) {
        return nullptr;
    }
    double minDist = std::numeric_limits<double>::infinity();
    BeliefNode *nnBel = currentBelief->nnBel_;
    long numTried = 0;
    for (BeliefNode *otherBelief : policy_->allNodes_) {
        if (currentBelief == otherBelief) {
            continue;
        }
        if (numTried >= model_->getMaxNnComparisons()) {
            break;
        } else {
            if (currentBelief->tNNComp_ < otherBelief->tLastAddedParticle_) {
                double distance = currentBelief->distL1Independent(otherBelief);
                if (distance < minDist) {
                    minDist = distance;
                    nnBel = otherBelief;
                }
            }
            numTried++;
        }
    }
    currentBelief->tNNComp_ = (double)std::clock() * 1000 / CLOCKS_PER_SEC;
    if (minDist > model_->getMaxNnDistance()) {
        return nullptr;
    }
    currentBelief->nnBel_ = nnBel;
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
        heuristicProbability_[i] = ((1 - heuristicExploreCoefficient_)
                * heuristicWeight_[i] / totWRollout
                + heuristicExploreCoefficient_ / 2);
        heuristicProbability_[i] *= heuristicUseCount_[i] / timeUsedPerHeuristic_[i];
        totP += heuristicProbability_[i];
    }
    for (int i = 0; i < 2; i++) {
        heuristicProbability_[i] /= totP;
    }
}

double Solver::runSim(long nSteps, std::vector<long> &changeTimes,
        std::vector<std::unique_ptr<State>> &trajSt,
        std::vector<std::unique_ptr<Action>> &trajAction,
        std::vector<std::unique_ptr<Observation>> &trajObs,
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
    long historiesPerStep = model_->getNumberOfHistoriesPerStep();
    long maximumDepth = model_->getMaximumDepth();
    double discFactor = model_->getDiscountFactor();
    double currDiscFactor = 1.0;
    double discountedTotalReward = 0.0;

    BeliefNode *currNode = policy_->getRoot();
    std::unique_ptr<State> currentState = model_->sampleAnInitState();
    trajSt.push_back(currentState->copy());

    cout << "Initial simulation state:" << endl;
    model_->drawSimulationState(currNode->getStates(), *currentState, cout);
    cout << endl;

    std::vector<long>::iterator itCh = changeTimes.begin();
    for (long timeStep = 0; timeStep < nSteps; timeStep++) {
        cout << "t-" << timeStep << endl;
        allStates_->createOrGetInfo(*currentState);
        if (itCh != changeTimes.end() && timeStep == *itCh) {
            // Apply the changes to the model.
            cout << "Model changing." << endl;

            chTimeStart = std::clock();
            model_->update(*itCh, allStates_.get());
            if (changes::hasFlag(allStates_->getInfo(*currentState)->changeFlags_,
                    ChangeFlags::DELETED)) {
                debug::show_message("ERROR: Current simulation state deleted. Exiting..");
                std::exit(1);
            }
            for (std::unique_ptr<State> &state2 : trajSt) {
                if (changes::hasFlag(
                        allStates_->getInfo(*state2)->changeFlags_,
                        ChangeFlags::DELETED)) {
                    std::ostringstream message;
                    message << "ERROR: Impossible simulation history! Includes ";
                    message << *state2;
                    debug::show_message(message.str());
                }
            }
            applyChanges();
            allStates_->resetAffectedStates();

            cout << "Changes complete" << endl;
            chTimeEnd = std::clock();

            *totChTime += ((chTimeEnd - chTimeStart) * 1000 / CLOCKS_PER_SEC);
            cout << "Total of " << *totChTime << " ms used for changes." << endl;

            itCh++;
        }
        impSolTimeStart = std::clock();
        improveSol(currNode, historiesPerStep, maximumDepth);
        impSolTimeEnd = std::clock();
        *totImpTime += ((impSolTimeEnd - impSolTimeStart) * 1000
                / CLOCKS_PER_SEC);

        Model::StepResult result = simAStep(currNode, *currentState);
        currentState = result.nextState->copy();

        trajAction.push_back(result.action->copy());
        trajObs.push_back(result.observation->copy());
        trajSt.push_back(result.nextState->copy());
        trajRew.push_back(result.reward);
        discountedTotalReward += currDiscFactor * result.reward;
        currDiscFactor = currDiscFactor * discFactor;
        cout << "Discount: " << currDiscFactor << "; Total Reward: "
             << discountedTotalReward << endl;
        if (result.isTerminal) {
            *actualNSteps = timeStep;
            cout << "Final state: " << endl;
            model_->drawSimulationState(std::vector<const State *>{},
                    *currentState, cout);
            break;
        }

        BeliefNode *nextNode = currNode->getChild(*result.action,
                    *result.observation);
        if (nextNode == nullptr) {
            nextNode = addChild(currNode, *result.action, *result.observation,
                    timeStep);
        }
        cout << "Simulation state: " << endl;
        model_->drawSimulationState(nextNode->getStates(),
                *currentState, cout);
        cout << endl;
        currNode = nextNode;
    }
    cout << "MDP heuristic used ";
    cout << heuristicUseCount_[ROLLOUT_RANDHEURISTIC] - 1 << " times; Took ";
    cout << timeUsedPerHeuristic_[ROLLOUT_RANDHEURISTIC] << "ms" << endl;
    cout << "NN heuristic used ";
    cout << heuristicUseCount_[ROLLOUT_POL] - 1 << " times; Took ";
    cout << timeUsedPerHeuristic_[ROLLOUT_POL] << "ms" << endl;
    return discountedTotalReward;
}

Model::StepResult Solver::simAStep(BeliefNode *currentBelief,
        State const &currentState) {
    // Particle sampling and variance estimation.
//    State const *state = currentBelief->sampleAParticle(randGen_)->getState();
//    cout << "Sampled particle: " << *state << endl;
//
//    double totalDistance = 0;
//    for (int i = 0; i < 100; i++) {
//        State const *s1 = currentBelief->sampleAParticle(randGen_)->getState();
//        State const *s2 = currentBelief->sampleAParticle(randGen_)->getState();
//        totalDistance += s1->distanceTo(*s2);
//    }
//    cout << "Est. mean inter-particle distance: " << totalDistance / 100 << endl;

//    cout << "Action children: " << endl;
//    std::multimap<double, Action const *> actionValues;
//    for (ActionNode *node : currentBelief->getMapping()->getChildren()) {
//        if (node == nullptr) {
//            continue;
//        }
//        if (std::isnan(node->meanQValue_)) {
//            debug::show_message("ERROR: NaN value!");
//        }
//        actionValues.emplace(node->meanQValue_, node->action_.get());
//    }
//    for (auto it = actionValues.rbegin(); it != actionValues.rend(); it++) {
//        cout << *it->second << " " << it->first << endl;
//    }

    std::unique_ptr<Action> action = currentBelief->getBestAction();
    if (action == nullptr) {
        action = currentBelief->getRolloutActions();
    }
    Model::StepResult result = model_->generateStep(currentState, *action);
    if (result.isTerminal) {
        cout << " Reached a terminal state." << endl;
    }
    cout << "Action: " << *result.action << endl;
    cout << "Transition: ";
    if (result.transitionParameters == nullptr) {
        cout << "NULL" << endl;
    } else {
        cout << *result.transitionParameters << endl;
    }
    cout << "Reward: " << result.reward << endl;
    cout << "Observation: " << *result.observation << endl;
    return result;
}

void Solver::improveSol(BeliefNode *startNode, long historiesPerStep,
        long maximumDepth) {
    if (startNode->getNParticles() == 0) {
        debug::show_message("ERROR: No particles in the BeliefNode!");
        std::exit(10);
    }

    std::vector<StateInfo *> nonTerminalStates;
    for (long index = 0; index < startNode->getNParticles(); index++) {
        HistoryEntry *entry = startNode->particles_.get(index);
        if (!model_->isTerminal(*entry->getState())) {
            nonTerminalStates.push_back(entry->stateInfo_);
        }
    }
    if (nonTerminalStates.empty()) {
        debug::show_message("ERROR: No non-terminal particles!");
        return;
    }

    std::vector<StateInfo *> samples;
    for (long i = 0; i < historiesPerStep; i++) {
        long index = std::uniform_int_distribution<long>(
                0, nonTerminalStates.size() - 1)(*randGen_);
        samples.push_back(nonTerminalStates[index]);
    }

    double disc = model_->getDiscountFactor();
    HistoryEntry *entry = startNode->particles_.get(0);
    long depth = entry->entryId_ + entry->owningSequence_->startDepth_;
    for (StateInfo *sample : samples) {
        singleSearch(startNode, sample, depth, disc, maximumDepth);
    }
}

BeliefNode *Solver::addChild(BeliefNode *currNode, Action const &action,
        Observation const &obs, long timeStep) {
    debug::show_message("WARNING: Adding particles due to depletion");
    BeliefNode *nextNode = policy_->createOrGetChild(currNode, action, obs);

    std::vector<State const *> particles;
    std::vector<HistoryEntry *>::iterator it;
    for (HistoryEntry *entry : currNode->particles_) {
        particles.push_back(entry->getState());
    }

    double discountFactor = model_->getDiscountFactor();
    double currentDiscount = std::pow(discountFactor, timeStep);
    // Attempt to generate particles for next state based on the current belief,
    // the observation, and the action.
    std::vector<std::unique_ptr<State>> nextParticles(
            model_->generateParticles(currNode, action, obs, particles));
    if (nextParticles.empty()) {
        debug::show_message("WARNING: Could not generate based on belief!");
        // If that fails, ignore the current belief.
        nextParticles = model_->generateParticles(currNode, action, obs);
    }
    if (nextParticles.empty()) {
        debug::show_message("ERROR: Failed to generate new particles!");
    }
    for (std::unique_ptr<State> &uniqueStatePtr : nextParticles) {
        StateInfo *stateInfo = allStates_->createOrGetInfo(*uniqueStatePtr);

        // Create a new history sequence and entry for the new particle.
        HistorySequence *histSeq = allHistories_->addNew(timeStep);
        HistoryEntry *histEntry = histSeq->addEntry(stateInfo, currentDiscount * discountFactor);
        histEntry->registerNode(nextNode);

//        State *state = stateInfo->getState();
//        if (!model_->isTerminal(*state)) {
//            histEntry->immediateReward_ = model_->getDefaultVal();
//            histEntry->totalDiscountedReward_ = (
//                    histEntry->discount_ * histEntry->immediateReward_);
//        }
        backup(histSeq);
    }
    return nextNode;
}

void Solver::applyChanges() {
    std::unordered_set<HistorySequence *> affectedSequences;
    for (StateInfo *stateInfo : allStates_->getAffectedStates()) {
        for (HistoryEntry *entry : stateInfo->usedInHistoryEntries_) {
            HistorySequence *sequence = entry->owningSequence_;
            long entryId = entry->entryId_;
            sequence->setChangeFlags(entryId, stateInfo->changeFlags_);
            if (changes::hasFlag(entry->changeFlags_, ChangeFlags::DELETED)) {
                if (entryId > 0) {
                    sequence->setChangeFlags(entryId - 1,
                            ChangeFlags::TRANSITION);
                }
            }
            if (changes::hasFlag(entry->changeFlags_,
                    ChangeFlags::OBSERVATION_BEFORE)) {
                if (entryId > 0) {
                    sequence->setChangeFlags(entryId - 1,
                            ChangeFlags::OBSERVATION);
                }
            }
            affectedSequences.insert(sequence);
        }
    }
    cout << "Updating " << affectedSequences.size() << " histories!" << endl;

    // Delete and remove any sequences where the first entry is now invalid.
    std::unordered_set<HistorySequence *>::iterator it = affectedSequences.begin();
    while (it != affectedSequences.end()) {
        HistorySequence *sequence = *it;
        undoBackup(sequence);
        if (changes::hasFlag(sequence->getEntry(0)->changeFlags_,
                ChangeFlags::DELETED)) {
            it = affectedSequences.erase(it);
            allHistories_->deleteHistorySequence(sequence->id_);
        } else {
            it++;
        }
    }

    // Revise all of the histories.
    historyCorrector_->reviseHistories(affectedSequences);

    // Clear flags and fix up all the sequences.
    for (HistorySequence *sequence : affectedSequences) {
        fixLinks(sequence);
        sequence->resetChangeFlags();
        if (sequence->isTerminal()) {
            backup(sequence);
        } else {
            continueSearch(sequence, model_->getDiscountFactor(),
                    model_->getMaximumDepth());
        }
    }
}

void Solver::fixLinks(HistorySequence *sequence) {
    if (sequence->invalidLinksStartId_ != -1) {
        std::vector<std::unique_ptr<HistoryEntry>>::iterator
        historyIterator = (sequence->histSeq_.begin()
                + sequence->invalidLinksStartId_);
        for ( ; (historyIterator + 1) != sequence->histSeq_.end();
                historyIterator++) {
            HistoryEntry *entry = historyIterator->get();
            HistoryEntry *nextEntry = (historyIterator + 1)->get();
            BeliefNode *nextNode = policy_->createOrGetChild(
                    entry->owningBeliefNode_, *entry->action_,
                    *entry->observation_);
            nextEntry->registerNode(nextNode);
        }
        sequence->invalidLinksStartId_ = -1;
    }
}

BeliefTree *Solver::getPolicy() {
    return policy_.get();
}

StatePool *Solver::getStatePool() {
    return allStates_.get();
}

Model *Solver::getModel() {
    return model_.get();
}

ActionPool *Solver::getActionPool() {
    return actionPool_.get();
}

ObservationPool *Solver::getObservationPool() {
    return observationPool_.get();
}
} /* namespace solver */
