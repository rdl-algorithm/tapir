#include "Solver.hpp"

#include <cmath>                        // for pow, exp
#include <cstdio>

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

#include <boost/regex.hpp>

using std::cout;
using std::endl;

namespace solver {
Solver::Solver(RandomGenerator *randGen, std::unique_ptr<Model> model) :
    randGen_(randGen),
    serializer_(nullptr),
    model_(std::move(model)),
    actionPool_(model_->createActionPool()),
    observationPool_(model_->createObservationPool()),
    allStates_(std::make_unique<StatePool>(model_->createStateIndex())),
    allHistories_(std::make_unique<Histories>()),
    policy_(std::make_unique<BeliefTree>()),
    historyCorrector_(model_->createHistoryCorrector()),
    searchStrategy_(nullptr),
    rolloutStrategy_(nullptr) {
}

// Default destructor
Solver::~Solver() {
}

/* ------------------ Initialization methods ------------------- */
void Solver::initialize() {
    actionPool_->observationPool_ = observationPool_.get();
    observationPool_->actionPool_ = actionPool_.get();
    policy_->setRoot(std::make_unique<BeliefNode>(
            actionPool_->createActionMapping(), 0));
    historyCorrector_->setSolver(this);
    searchStrategy_ = model_->createSearchStrategy(this);
    rolloutStrategy_ = model_->createRolloutStrategy(this);
}
void Solver::setSerializer(std::unique_ptr<Serializer> serializer) {
    serializer_ = std::move(serializer);
}
void Solver::saveStateTo(std::ostream &os) {
    serializer_->save(os);
}
void Solver::loadStateFrom(std::istream &is) {
    serializer_->load(is);
}

/* ------------------ Solution methods ------------------- */
void Solver::genPol(long historiesPerStep, long maximumDepth) {
    // Start expanding the tree.
    for (long i = 0; i < historiesPerStep; i++) {
        singleSearch(maximumDepth);
    }
}


double Solver::runSim(long nSteps, long historiesPerStep,
        std::vector<long> &changeTimes,
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
    double chTimeStart, chTimeEnd, impSolTimeStart, impSolTimeEnd;
    *actualNSteps = nSteps;
    long maximumDepth = model_->getMaximumDepth();
    double discFactor = model_->getDiscountFactor();
    double currDiscFactor = 1.0;
    double discountedTotalReward = 0.0;

    BeliefNode *currNode = policy_->getRoot();
    std::unique_ptr<State> currentState = model_->sampleAnInitState();
    trajSt.push_back(currentState->copy());
    std::vector<long>::iterator itCh = changeTimes.begin();
    for (long timeStep = 0; timeStep < nSteps; timeStep++) {
        std::stringstream prevStream;
        if (model_->hasVerboseOutput()) {
            cout << endl << endl << "t-" << timeStep << endl;
            prevStream << "Before:" << endl;
            printBelief(currNode, prevStream);
        }

        allStates_->createOrGetInfo(*currentState);
        if (itCh != changeTimes.end() && timeStep == *itCh) {
            // Apply the changes to the model.
            if (model_->hasVerboseOutput()) {
                cout << "Model changing." << endl;
            }

            chTimeStart = abt::clock_ms();
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
                    debug::show_message(message.str(), true, false);
                }
            }
            applyChanges();
            allStates_->resetAffectedStates();
            chTimeEnd = abt::clock_ms();
            *totChTime += chTimeEnd - chTimeStart;

            if (model_->hasVerboseOutput()) {
                cout << "Changes complete" << endl;
                cout << "Total of " << *totChTime << " ms used for changes." << endl;
            }
            itCh++;
        }
        impSolTimeStart = abt::clock_ms();
        improveSol(currNode, historiesPerStep, maximumDepth);
        impSolTimeEnd = abt::clock_ms();
        *totImpTime += impSolTimeEnd - impSolTimeStart;

        if (model_->hasVerboseOutput()) {
            cout << "State: " << *currentState << endl;
            cout << "Heuristic Value: " << model_->getHeuristicValue(*currentState) << endl;
            cout << "Belief #" << currNode->getId() << endl;
            model_->drawSimulationState(currNode, *currentState, cout);

            std::stringstream newStream;
            newStream << "After:" << endl;
            printBelief(currNode, newStream);
            while (prevStream.good() || newStream.good()) {
                std::string s1, s2;
                std::getline(prevStream, s1);
                std::getline(newStream, s2);
                boost::regex rgx("\\x1b\\[[0-9;]*m");
                std::string s1Text = boost::regex_replace(s1, rgx, "");
                std::string s2Text = boost::regex_replace(s2, rgx, "");
                cout << s1 << std::setw(40 - s1Text.size()) << "";
                cout << s2 << std::setw(40 - s2Text.size()) << "";
                cout << endl;
            }
        }

        Model::StepResult result = simAStep(currNode, *currentState);
        currentState = result.nextState->copy();
        trajAction.push_back(result.action->copy());
        trajObs.push_back(result.observation->copy());
        trajSt.push_back(result.nextState->copy());
        trajRew.push_back(result.reward);
        discountedTotalReward += currDiscFactor * result.reward;
        currDiscFactor = currDiscFactor * discFactor;

        if (model_->hasVerboseOutput()) {
            if (result.isTerminal) {
                cout << " Reached a terminal state!" << endl;
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
            cout << "Discount: " << currDiscFactor << "; Total Reward: ";
            cout << discountedTotalReward << endl;
        }

        BeliefNode *nextNode = currNode->getChild(*result.action,
                    *result.observation);
        if (nextNode == nullptr) {
            nextNode = addChild(currNode, *result.action, *result.observation, timeStep);
        }
        currNode = nextNode;
        if (result.isTerminal) {
            *actualNSteps = timeStep;
            break;
        }
    }
    if (model_->hasVerboseOutput()) {
        cout << endl << endl << "Final State:" << endl;
        cout << *currentState << endl;
        cout << "Belief #" << currNode->getId() << endl;
        model_->drawSimulationState(currNode, *currentState, cout);
    }
    return discountedTotalReward;
}

/* ------------------ Simple getters. ------------------- */
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

/* ------------------ Episode sampling methods ------------------- */
void Solver::singleSearch(long maximumDepth) {
    StateInfo *stateInfo = allStates_->createOrGetInfo(
            *model_->sampleAnInitState());
    singleSearch(policy_->getRoot(), stateInfo, 0, maximumDepth);
}

void Solver::singleSearch(BeliefNode *startNode, StateInfo *startStateInfo,
        long startDepth, long maximumDepth) {
    HistorySequence *sequence = allHistories_->addSequence(startDepth);
    sequence->addEntry(startStateInfo);
    sequence->registerStartingNode(startNode);
    continueSearch(sequence, maximumDepth);
}

void Solver::continueSearch(HistorySequence *sequence, long maximumDepth) {
    if (model_->isTerminal(*sequence->getLastEntry()->getState())) {
        debug::show_message("WARNING: Attempted to continue sequence"
                " from a terminal state.");
        return;
    }
    SearchStatus status = SearchStatus::UNINITIALIZED;

    std::unique_ptr<SearchInstance> searchInstance = nullptr;
    searchInstance = searchStrategy_->createSearchInstance(sequence,
            maximumDepth);
    status = searchInstance->initialize();
    if (status != SearchStatus::INITIAL) {
        debug::show_message("WARNING: Search algorithm could not initialize!?");
    }
    status = searchInstance->extendSequence();
    if (status == SearchStatus::REACHED_ROLLOUT_NODE) {
        searchInstance = rolloutStrategy_->createSearchInstance(sequence,
                maximumDepth);
        status = searchInstance->initialize();
        if (status != SearchStatus::INITIAL) {
            debug::show_message("WARNING: Rollout algorithm could not initialize!?");
        }
        status = searchInstance->extendSequence();
    }
    if (status == SearchStatus::ROLLOUT_COMPLETE || status == SearchStatus::HIT_DEPTH_LIMIT) {
        HistoryEntry *lastEntry = sequence->getLastEntry();
        State const *lastState = lastEntry->getState();
        if (model_->isTerminal(*lastState)) {
            debug::show_message("ERROR: Status should be terminal, but it wasn't!");
        }
        lastEntry->rewardFromHere_ = model_->getHeuristicValue(*lastState);
    } else if (status == SearchStatus::HIT_TERMINAL_STATE) {
    } else {
        debug::show_message("ERROR: Search failed!?");
        return;
    }
    // Register and backup.
    sequence->registerRestOfSequence(true, policy_.get());
    backup(sequence, true);
}

/* ------------------ Tree backup methods ------------------- */
void Solver::backup(HistorySequence *sequence, bool backingUp) {
    sequence->backupIsValid(backingUp);
    double discountFactor = model_->getDiscountFactor();

    std::vector<std::unique_ptr<HistoryEntry>>::reverse_iterator itHist = (
                sequence->histSeq_.rbegin());
    // Retrieve the value of the last entry.
    double totalReward = (*itHist)->rewardFromHere_;
    (*itHist)->hasBeenBackedUp_ = backingUp;
    itHist++;

    bool addFullReward = true;
    // Normal update for the rest of the sequence.
    for (; itHist != sequence->histSeq_.rend(); itHist++) {
        HistoryEntry *entry = itHist->get();
        BeliefNode *node = entry->associatedBeliefNode_;

        // Apply the discount
        totalReward *= discountFactor;
        // Calculate the reward from this entry.
        entry->rewardFromHere_ = totalReward = entry->reward_ + totalReward;

        long deltaNParticles = 1;
        double deltaQ = entry->reward_;
        if (addFullReward) {
            deltaQ = totalReward;
        }
        // If we're undoing it, we negate the values.
        if (!backingUp) {
            deltaNParticles = -deltaNParticles;
            deltaQ = -deltaQ;
        }

        ActionNode *actionNode = node->getMapping()->getActionNode(*entry->action_);
        actionNode->changeTotalQValue(deltaQ, deltaNParticles);
        if (addFullReward) {
            actionNode->getChild(*entry->observation_)->recalculateQValue();
            addFullReward = false;
        } else {
            actionNode->updateChildQValue(*entry->observation_,
                    model_->getDiscountFactor(), deltaNParticles);
        }
        actionNode->recalculateQValue();
        entry->hasBeenBackedUp_ = backingUp;
    }
    // The belief node also needs to recalculate its q value.
    sequence->getFirstEntry()->associatedBeliefNode_->recalculateQValue();
}

/* ------------------ Simulation methods ------------------- */
Model::StepResult Solver::simAStep(BeliefNode *currentBelief,
        State const &currentState) {
    std::unique_ptr<Action> action = currentBelief->getBestAction();
    if (action == nullptr) {
        debug::show_message("WARNING: No actions evaluated! Selecting a random action...");
        action = currentBelief->getMapping()->getRandomRolloutAction();
    }
    Model::StepResult result = model_->generateStep(currentState, *action);
    return result;
}

void Solver::improveSol(BeliefNode *startNode, long historiesPerStep,
        long maximumDepth) {
    if (startNode->getNumberOfParticles() == 0) {
        debug::show_message("ERROR: No particles in the BeliefNode!", true, false);
        std::exit(10);
    }

    std::vector<StateInfo *> nonTerminalStates;
    for (long index = 0; index < startNode->getNumberOfParticles(); index++) {
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

    HistoryEntry *entry = startNode->particles_.get(0);
    long depth = entry->entryId_ + entry->owningSequence_->startDepth_;
    for (StateInfo *sample : samples) {
        singleSearch(startNode, sample, depth, maximumDepth);
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
        HistorySequence *histSeq = allHistories_->addSequence(timeStep);
        HistoryEntry *histEntry = histSeq->addEntry(stateInfo);
        State const *state = stateInfo->getState();
        if (!model_->isTerminal(*state)) {
            // Use the heuristic value for non-terminal particles.
            histEntry->rewardFromHere_ = model_->getHeuristicValue(*state);
        }
        // Register and backup
        histSeq->registerStartingNode(nextNode);
        backup(histSeq, true);
    }
    return nextNode;
}

/* -------------- Methods for handling model changes --------------- */
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
    if (model_->hasVerboseOutput()) {
        cout << "Updating " << affectedSequences.size() << " histories!";
        cout << endl;
    }

    // Delete and remove any sequences where the first entry is now invalid.
    std::unordered_set<HistorySequence *>::iterator it = affectedSequences.begin();
    while (it != affectedSequences.end()) {
        HistorySequence *sequence = *it;

        // Undo backup and deregister.
        backup(sequence, false);
        sequence->registerRestOfSequence(false, nullptr);

        if (changes::hasFlag(sequence->getFirstEntry()->changeFlags_,
                ChangeFlags::DELETED)) {
            it = affectedSequences.erase(it);
            // Deregister the starting node as well.
            sequence->registerStartingNode(nullptr);
            // Now remove the sequence entirely.
            allHistories_->deleteSequence(sequence->id_);
        } else {
            it++;
        }
    }

    // Revise all of the histories.
    historyCorrector_->reviseHistories(affectedSequences);

    // Clear flags and fix up all the sequences.
    for (HistorySequence *sequence : affectedSequences) {
        sequence->resetChangeFlags();
        HistoryEntry *lastEntry = sequence->getLastEntry();
        State const *lastState = lastEntry->getState();
        // If it didn't end in a terminal state, we apply the heuristic.
        if (!model_->isTerminal(*lastState)) {
            lastEntry->rewardFromHere_ = model_->getHeuristicValue(*lastState);
        }

        // Now we register and then backup.
        sequence->registerRestOfSequence(true, policy_.get());
        backup(sequence, true);
    }
}

/* ------------------ Display methods  ------------------- */
void Solver::printBelief(BeliefNode *belief, std::ostream &os) {
    os << belief->getQValue();
    os << " from " << belief->getNumberOfParticles() << " p." << endl;
    os << belief->getNumberOfSequenceEdges() << " edges" << endl;
    os << "Action children: " << endl;
    std::multimap<double, solver::ActionMappingEntry const *> actionValues;
    for (solver::ActionMappingEntry const *entry : belief->getMapping()->getChildEntries()) {
        actionValues.emplace(entry->getActionNode()->getQValue(), entry);
    }
    for (auto it = actionValues.rbegin(); it != actionValues.rend(); it++) {
        abt::print_double(it->first, os, 8, 2,
                std::ios_base::fixed | std::ios_base::showpos);
        os << ": ";
        std::ostringstream sstr;
        sstr << *it->second->getAction();
        abt::print_with_width(sstr.str(), os, 17);
        abt::print_with_width(it->second->getActionNode()->getNParticles(),
                os, 8);
        os << endl;
    }
}
} /* namespace solver */
