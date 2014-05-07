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

#include "backpropagation/BackpropagationStrategy.hpp"

#include "changes/ChangeFlags.hpp"               // for ChangeFlags, ChangeFlags::UNCHANGED, ChangeFlags::ADDOBSERVATION, ChangeFlags::ADDOBSTACLE, ChangeFlags::ADDSTATE, ChangeFlags::DELSTATE, ChangeFlags::REWARD, ChangeFlags::TRANSITION
#include "changes/HistoryCorrector.hpp"

#include "mappings/ActionMapping.hpp"
#include "mappings/ActionPool.hpp"
#include "mappings/ObservationMapping.hpp"
#include "mappings/ObservationPool.hpp"

#include "search/SearchStatus.hpp"
#include "search/SearchStrategy.hpp"

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
    randGen_(randGen),
    serializer_(nullptr),
    model_(std::move(model)),
    statePool_(nullptr),
    histories_(nullptr),
    policy_(nullptr),
    actionPool_(nullptr),
    observationPool_(nullptr),
    historyCorrector_(nullptr),
    selectionStrategy_(nullptr),
    rolloutStrategy_(nullptr),
    backpropagationStrategy_(nullptr) {
}

// Default destructor
Solver::~Solver() {
}

/* ------------------ Simple getters. ------------------- */
BeliefTree *Solver::getPolicy() const {
    return policy_.get();
}
StatePool *Solver::getStatePool() const {
    return statePool_.get();
}
Model *Solver::getModel() const {
    return model_.get();
}
ActionPool *Solver::getActionPool() const {
    return actionPool_.get();
}
ObservationPool *Solver::getObservationPool() const {
    return observationPool_.get();
}

/* ------------------ Initialization methods ------------------- */
void Solver::initializeEmpty() {
    // Basic initialization.
    initialize();

    // Create new instances of these.
    actionPool_ = model_->createActionPool(this);
    observationPool_ = model_->createObservationPool(this);

    // Initialize the root node properly.
    BeliefNode *rootPtr = policy_->getRoot();
    rootPtr->setHistoricalData(model_->createRootHistoricalData());
    rootPtr->setMapping(actionPool_->createActionMapping());
    rootPtr->getMapping()->initialize();
}

void Solver::setSerializer(std::unique_ptr<Serializer> serializer) {
    serializer_ = std::move(serializer);
}
void Solver::saveStateTo(std::ostream &os) const {
    serializer_->save(os);
}
void Solver::loadStateFrom(std::istream &is) {
    serializer_->load(is);
}

/* ------------------- Policy mutators ------------------- */
void Solver::improvePolicy(long numberOfHistories, long maximumDepth) {
	std::vector<StateInfo *> states;
	// Generate the initial states.
    for (long i = 0; i < numberOfHistories; i++) {
    	states.push_back(statePool_->createOrGetInfo(
    			*model_->sampleAnInitState()));
    }
    multipleSearches(policy_->getRoot(), states, maximumDepth);
}

void Solver::improvePolicy(BeliefNode *startNode, long numberOfHistories,
        long maximumDepth) {
    if (startNode->getNumberOfParticles() == 0) {
        debug::show_message("ERROR: No particles in the BeliefNode!");
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
        std::exit(11);
    }

    std::vector<StateInfo *> samples;
    for (long i = 0; i < numberOfHistories; i++) {
        long index = std::uniform_int_distribution<long>(
                0, nonTerminalStates.size() - 1)(*randGen_);
        samples.push_back(nonTerminalStates[index]);
    }

    multipleSearches(startNode, samples, maximumDepth);
}

void Solver::applyChanges() {
    std::unordered_set<HistorySequence *> affectedSequences;
    for (StateInfo *stateInfo : statePool_->getAffectedStates()) {
        for (HistoryEntry *entry : stateInfo->usedInHistoryEntries_) {
            HistorySequence *sequence = entry->owningSequence_;
            long entryId = entry->entryId_;
            sequence->setChangeFlags(entryId, stateInfo->changeFlags_);
            if (changes::has_flag(entry->changeFlags_, ChangeFlags::DELETED)) {
                if (entryId > 0) {
                    sequence->setChangeFlags(entryId - 1,
                            ChangeFlags::TRANSITION);
                }
            }
            if (changes::has_flag(entry->changeFlags_,
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
        sequence->registerWith(nullptr, nullptr);

        if (changes::has_flag(sequence->getFirstEntry()->changeFlags_,
                ChangeFlags::DELETED)) {
            it = affectedSequences.erase(it);
            // Now remove the sequence entirely.
            histories_->deleteSequence(sequence->id_);
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
        sequence->registerWith(sequence->getFirstEntry()->associatedBeliefNode_,
                    policy_.get());
        backup(sequence, true);
    }

    // Reset the set of affected states in the pool.
    statePool_->resetAffectedStates();
}

BeliefNode *Solver::addChild(BeliefNode *currNode, Action const &action,
        Observation const &obs) {
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
        StateInfo *stateInfo = statePool_->createOrGetInfo(*uniqueStatePtr);

        // Create a new history sequence and entry for the new particle.
        HistorySequence *histSeq = histories_->createSequence();
        HistoryEntry *histEntry = histSeq->addEntry(stateInfo);
        State const *state = stateInfo->getState();
        if (!model_->isTerminal(*state)) {
            // Use the heuristic value for non-terminal particles.
            histEntry->rewardFromHere_ = model_->getHeuristicValue(*state);
        }
        // Register and backup
        histSeq->registerWith(nextNode, policy_.get());
        backup(histSeq, true);
    }
    return nextNode;
}

/* ------------------ Display methods  ------------------- */
void Solver::printBelief(BeliefNode *belief, std::ostream &os) {
    os << belief->getQValue();
    os << " from " << belief->getNumberOfParticles() << " p.";
    os << " with " << belief->getNumberOfStartingSequences() << " starts.";
    os << endl;
    os << "Action children: " << endl;
    std::multimap<double, solver::ActionMappingEntry const *> actionValues;
    for (solver::ActionMappingEntry const *entry : belief->getMapping()->getVisitedEntries()) {
        actionValues.emplace(entry->getMeanQValue(), entry);
    }
    for (auto it = actionValues.rbegin(); it != actionValues.rend(); it++) {
        abt::print_double(it->first, os, 8, 2,
                std::ios_base::fixed | std::ios_base::showpos);
        os << ": ";
        std::ostringstream sstr;
        sstr << *it->second->getAction();
        abt::print_with_width(sstr.str(), os, 17);
        abt::print_with_width(it->second->getVisitCount(), os, 8);
        os << endl;
    }
}

/* ------------------ Simulation methods ------------------- */
double Solver::runSimulation(long nSteps, long historiesPerStep,
        std::vector<long> &changeTimes,
        std::vector<std::unique_ptr<State>> &trajSt,
        std::vector<std::unique_ptr<Action>> &trajAction,
        std::vector<std::unique_ptr<Observation>> &trajObs,
        std::vector<double> &trajRew,
        long *actualNSteps,
        double *totChTime,
        double *totImpTime) {
    trajSt.clear();
    trajAction.clear();
    trajObs.clear();
    trajRew.clear();

    *totChTime = 0.0;
    *totImpTime = 0.0;
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
            cout << "State: " << *currentState << endl;
            cout << "Heuristic Value: " << model_->getHeuristicValue(*currentState) << endl;
            cout << "Belief #" << currNode->getId() << endl;

            solver::HistoricalData *data = currNode->getHistoricalData();
            if (data != nullptr) {
                cout << endl;
                cout << *data;
                cout << endl;
            }

            model_->drawSimulationState(currNode, *currentState, cout);

            prevStream << "Before:" << endl;
            printBelief(currNode, prevStream);
        }

        statePool_->createOrGetInfo(*currentState);

        // If the model is changing, handle the changes.
        if (itCh != changeTimes.end() && timeStep == *itCh) {
            if (model_->hasVerboseOutput()) {
                cout << "Model changing." << endl;
            }
            double chTimeStart = abt::clock_ms();
            // Apply all the changes!
            handleChanges(timeStep, *currentState, trajSt);
            double chTimeEnd = abt::clock_ms();
            *totChTime += chTimeEnd - chTimeStart;
            if (model_->hasVerboseOutput()) {
                cout << "Changes complete" << endl;
                cout << "Total of " << *totChTime << " ms used for changes." << endl;
            }
            itCh++;
        }

        // Improve the policy
        double impSolTimeStart = abt::clock_ms();
        improvePolicy(currNode, historiesPerStep, maximumDepth);
        double impSolTimeEnd = abt::clock_ms();
        *totImpTime += impSolTimeEnd - impSolTimeStart;

        if (model_->hasVerboseOutput()) {
            std::stringstream newStream;
            newStream << "After:" << endl;
            printBelief(currNode, newStream);
            while (prevStream.good() || newStream.good()) {
                std::string s1, s2;
                std::getline(prevStream, s1);
                std::getline(newStream, s2);
                cout << s1 << std::setw(40 - s1.size()) << "";
                cout << s2 << std::setw(40 - s2.size()) << "";
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
                cout << "Reached a terminal state!" << endl;
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
            debug::show_message("ERROR: Resulting belief has zero particles!!");
            nextNode = addChild(currNode, *result.action, *result.observation);
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


/* ============================ PRIVATE ============================ */


/* ------------------ Initialization methods ------------------- */
void Solver::initialize() {
    // Core data structures
    statePool_ = std::make_unique<StatePool>(model_->createStateIndex());
    histories_ = std::make_unique<Histories>();
    policy_ = std::make_unique<BeliefTree>(this);

    // Serializable model-specific customizations
    actionPool_ = nullptr;
    observationPool_ = nullptr;

    // Possible model-specific customizations
    historyCorrector_ = model_->createHistoryCorrector(this);
    selectionStrategy_ = model_->createSelectionStrategy(this);
    rolloutStrategy_ = model_->createRolloutStrategy(this);
    backpropagationStrategy_ = model_->createBackpropagationStrategy(this);
}

/* ------------------ Episode sampling methods ------------------- */
void Solver::multipleSearches(BeliefNode *startNode, std::vector<StateInfo *> states,
		long maximumDepth) {
	for (StateInfo *stateInfo : states) {
		singleSearch(startNode, stateInfo, maximumDepth);
	}
}

void Solver::singleSearch(BeliefNode *startNode, StateInfo *startStateInfo,
		long maximumDepth) {
    HistorySequence *sequence = histories_->createSequence();
    sequence->addEntry(startStateInfo);
    sequence->getFirstEntry()->associatedBeliefNode_ = startNode;
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
    searchInstance = selectionStrategy_->createSearchInstance(sequence,
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
            debug::show_message("ERROR: Terminal state, but the search status"
                    " didn't reflect this!");
        }
        // Use the heuristic estimate.
        lastEntry->rewardFromHere_ = model_->getHeuristicValue(*lastState);
    } else if (status == SearchStatus::HIT_TERMINAL_STATE) {
        // Don't do anything for a terminal state.
    } else {
        debug::show_message("ERROR: Search failed!?");
        return;
    }
    // Register and backup.
    sequence->registerWith(sequence->getFirstEntry()->associatedBeliefNode_,
            policy_.get());
    backup(sequence, true);
}

/* ------------------ Tree backup methods ------------------- */
void Solver::calculateRewards(HistorySequence *sequence) {
    double discountFactor = model_->getDiscountFactor();
    std::vector<std::unique_ptr<HistoryEntry>>::reverse_iterator itHist = (
                sequence->entrySequence_.rbegin());
    // Retrieve the value of the last entry.
    double totalReward = (*itHist)->rewardFromHere_;
    itHist++;
    for (; itHist != sequence->entrySequence_.rend(); itHist++) {
           HistoryEntry *entry = itHist->get();
           // Apply the discount
           totalReward *= discountFactor;
           // Include the reward from this entry.
           entry->rewardFromHere_ = totalReward = entry->reward_ + totalReward;
    }
}

void Solver::backup(HistorySequence *sequence, bool backingUp) {
    sequence->testBackup(backingUp);
    calculateRewards(sequence);
    backpropagationStrategy_->propagate(sequence, !backingUp);
}

/* ------------------ Simulation methods ------------------- */
Model::StepResult Solver::simAStep(BeliefNode *currentBelief,
        State const &currentState) {
    std::unique_ptr<Action> action = currentBelief->getRecommendedAction();
    if (action == nullptr) {
        debug::show_message("ERROR: Could not choose an action!");
        std::exit(-2);
    }
    Model::StepResult result = model_->generateStep(currentState, *action);
    return result;
}

/* -------------- Methods for handling model changes --------------- */
void Solver::handleChanges(long timeStep,
        State const &currentState,
        std::vector<std::unique_ptr<State>> &stateHistory) {
    // Mark the states that need changing.
    model_->update(timeStep, statePool_.get());

    // Check if the model changes have invalidated our history...
    if (changes::has_flag(statePool_->getInfo(currentState)->changeFlags_,
            ChangeFlags::DELETED)) {
        debug::show_message(
                "ERROR: Current simulation state deleted. Exiting..");
        std::exit(1);
    }
    for (std::unique_ptr<State> &state2 : stateHistory) {
        if (changes::has_flag(statePool_->getInfo(*state2)->changeFlags_,
                ChangeFlags::DELETED)) {
            std::ostringstream message;
            message << "ERROR: Impossible simulation history! Includes ";
            message << *state2;
            debug::show_message(message.str());
        }
    }

    // Apply the changes
    applyChanges();
}
} /* namespace solver */
