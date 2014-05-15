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
#include "abstract-problem/ModelChange.hpp"                    // for Model::StepResult, Model
#include "abstract-problem/Observation.hpp"              // for Observation
#include "abstract-problem/State.hpp"                    // for State, operator<<

#include "changes/ChangeFlags.hpp"               // for ChangeFlags, ChangeFlags::UNCHANGED, ChangeFlags::ADDOBSERVATION, ChangeFlags::ADDOBSTACLE, ChangeFlags::ADDSTATE, ChangeFlags::DELSTATE, ChangeFlags::REWARD, ChangeFlags::TRANSITION
#include "changes/HistoryCorrector.hpp"

#include "mappings/actions/ActionMapping.hpp"
#include "mappings/actions/ActionPool.hpp"
#include "mappings/observations/ObservationMapping.hpp"
#include "mappings/observations/ObservationPool.hpp"

#include "search/SearchStatus.hpp"
#include "search/search_interface.hpp"

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
            beliefEstimationStrategy_(nullptr),
            nodesToBackup_() {
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
BeliefEstimationStrategy *Solver::getBeliefEstimationStrategy() const {
    return beliefEstimationStrategy_.get();
}

/* ------------------ Initialization methods ------------------- */
void Solver::initializeEmpty() {
    // Basic initialization.
    initialize();

    // Create new instances of these.
    actionPool_ = model_->createActionPool(this);
    observationPool_ = model_->createObservationPool(this);

    // Initialize the root node properly.
    policy_->initializeRoot();
}

Serializer *Solver::getSerializer() {
    return serializer_.get();
}
void Solver::setSerializer(std::unique_ptr<Serializer> serializer) {
    serializer_ = std::move(serializer);
}

/* ------------------- Policy mutators ------------------- */
void Solver::improvePolicy(long numberOfHistories, long maximumDepth) {
    if (numberOfHistories < 0) {
        numberOfHistories = model_->getNumberOfHistoriesPerStep();
    }
    if (maximumDepth < 0) {
        maximumDepth = model_->getMaximumDepth();
    }

    // Generate the initial states.
    std::vector<StateInfo *> states;
    for (long i = 0; i < numberOfHistories; i++) {
        states.push_back(statePool_->createOrGetInfo(*model_->sampleAnInitState()));
    }
    multipleSearches(policy_->getRoot(), states, maximumDepth);
}

void Solver::improvePolicy(BeliefNode *startNode, long numberOfHistories, long maximumDepth) {
    if (numberOfHistories < 0) {
        numberOfHistories = model_->getNumberOfHistoriesPerStep();
    }
    if (maximumDepth < 0) {
        maximumDepth = model_->getMaximumDepth();
    }
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
        long index = std::uniform_int_distribution<long>(0, nonTerminalStates.size() - 1)(
                *randGen_);
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
                    sequence->setChangeFlags(entryId - 1, ChangeFlags::TRANSITION);
                }
            }
            if (changes::has_flag(entry->changeFlags_, ChangeFlags::OBSERVATION_BEFORE)) {
                if (entryId > 0) {
                    sequence->setChangeFlags(entryId - 1, ChangeFlags::OBSERVATION);
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
        if (changes::has_flag(sequence->getFirstEntry()->changeFlags_, ChangeFlags::DELETED)) {
            it = affectedSequences.erase(it);
            // Now remove the sequence entirely.
            negateSequence(sequence);
            histories_->deleteSequence(sequence->id_);
        } else {
            it++;
        }
    }

    // Revise all of the histories.
    historyCorrector_->reviseHistories(affectedSequences);

    // Clear the change flags for all of the affected sequences.
    for (HistorySequence *sequence : affectedSequences) {
        sequence->resetChangeFlags();
    }

    // Reset the set of affected states in the pool.
    statePool_->resetAffectedStates();
    // Backup all the changes simultaneously.
    doBackup();
}

BeliefNode *Solver::replenishChild(BeliefNode *currNode, Action const &action,
        Observation const &obs, long minParticleCount) {
    if (minParticleCount < 0) {
        minParticleCount = model_->getMinParticleCount();
    }
    BeliefNode *nextNode = policy_->createOrGetChild(currNode, action, obs);
    long particleCount = nextNode->getNumberOfParticles();
    long deficit = minParticleCount - particleCount;
    if (deficit <= 0) {
        return nextNode;
    }

    if (model_->hasVerboseOutput()) {
        cout << "Replenishing particles...          ";
        cout.flush();
    }

    std::vector<State const *> particles;
    std::vector<HistoryEntry *>::iterator it;
    for (HistoryEntry *entry : currNode->particles_) {
        State const *state = entry->getState();
        if (!model_->isTerminal(*state)) {
            particles.push_back(state);
        }
    }

    // Attempt to generate particles for next state based on the current belief,
    // the observation, and the action.
    std::vector<std::unique_ptr<State>> nextParticles = (model_->generateParticles(currNode, action,
            obs, deficit, particles));
    if (nextParticles.empty()) {
        debug::show_message("WARNING: Could not generate based on belief!");
        // If that fails, ignore the current belief.
        nextParticles = model_->generateParticles(currNode, action, obs, deficit);
    }
    if (nextParticles.empty()) {
        debug::show_message("ERROR: Failed to generate new particles!");
        return nullptr;
    }

    for (std::unique_ptr<State> &uniqueStatePtr : nextParticles) {
        StateInfo *stateInfo = statePool_->createOrGetInfo(*uniqueStatePtr);

        // Create a new history sequence and entry for the new particle.
        HistorySequence *histSeq = histories_->createSequence();
        HistoryEntry *histEntry = histSeq->addEntry(stateInfo);
        histEntry->registerNode(nextNode);
    }
    if (model_->hasVerboseOutput()) {
        cout << "Done" << std::endl;
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
        abt::print_double(it->first, os, 8, 2, std::ios_base::fixed | std::ios_base::showpos);
        os << ": ";
        std::ostringstream sstr;
        sstr << *it->second->getAction();
        abt::print_with_width(sstr.str(), os, 17);
        abt::print_with_width(it->second->getVisitCount(), os, 8);
        os << endl;
    }
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
    beliefEstimationStrategy_ = model_->createBeliefEstimationStrategy(this);
}

/* ------------------ Episode sampling methods ------------------- */
void Solver::multipleSearches(BeliefNode *startNode, std::vector<StateInfo *> states,
        long maximumDepth) {
    if (maximumDepth < 0) {
        maximumDepth = model_->getMaximumDepth();
    }
    for (StateInfo *stateInfo : states) {
        singleSearch(startNode, stateInfo, maximumDepth);
        // Backup after every search to keep an accurate idea of where to explore.
        doBackup();
    }
}

void Solver::singleSearch(BeliefNode *startNode, StateInfo *startStateInfo, long maximumDepth) {
    if (maximumDepth < 0) {
        maximumDepth = model_->getMaximumDepth();
    }
    HistorySequence *sequence = histories_->createSequence();
    sequence->addEntry(startStateInfo);
    sequence->getFirstEntry()->registerNode(startNode);
    continueSearch(sequence, maximumDepth);
}

void Solver::continueSearch(HistorySequence *sequence, long maximumDepth) {
    if (maximumDepth < 0) {
        maximumDepth = model_->getMaximumDepth();
    }
    if (model_->isTerminal(*sequence->getLastEntry()->getState())) {
        debug::show_message("WARNING: Attempted to continue sequence"
                " from a terminal state.");
        return;
    }
    std::unique_ptr<SearchInstance> searchInstance = selectionStrategy_->createSearchInstance(
            sequence, maximumDepth);
    searchInstance->extendSequence();
    SearchStatus status = searchInstance->getStatus();
    if (status == SearchStatus::UNINITIALIZED) {
        debug::show_message("WARNING: Search algorithm could not initialize!?");
    }
    if (status == SearchStatus::REACHED_ROLLOUT_NODE) {
        searchInstance = rolloutStrategy_->createSearchInstance(sequence, maximumDepth);
        searchInstance->extendSequence();
        status = searchInstance->getStatus();
        if (status == SearchStatus::UNINITIALIZED) {
            debug::show_message("WARNING: Search algorithm could not initialize!?");
        }
    }
    if (status == SearchStatus::ROLLOUT_COMPLETE || status == SearchStatus::HIT_DEPTH_LIMIT) {
        HistoryEntry *lastEntry = sequence->getLastEntry();
        State const *lastState = lastEntry->getState();
        if (model_->isTerminal(*lastState)) {
            debug::show_message("ERROR: Terminal state, but the search status"
                    " didn't reflect this!");
        }
    } else if (status == SearchStatus::HIT_TERMINAL_STATE) {
        // Don't do anything for a terminal state.
    } else {
        debug::show_message("ERROR: Search failed!?");
        return;
    }
}

/* ------------------ Tree backup methods ------------------- */
void Solver::negateSequence(HistorySequence *sequence) {
    auto it = sequence->entrySequence_.cbegin();
    while (true) {
        // Update the immediate reward and the visit counts.
        updateImmediate((*it)->getAssociatedBeliefNode(), *(*it)->action_, *(*it)->observation_,
                -(*it)->immediateReward_, -1);

        it++; // Increment

        // Update the estimate of the next node's value.
        if ((*it)->action_ == nullptr) {
            // Last entry - negate the heuristic estimate
            updateEstimate((*it)->getAssociatedBeliefNode(), -(*it)->immediateReward_, 0);
            break;
        } else {
            // Not the last entry - negate 1x continuation.
            updateEstimate((*it)->getAssociatedBeliefNode(), 0, -1);
        }
    }
}

bool Solver::isBackedUp() const {
    return nodesToBackup_.empty();
}

/* ------------------ Value updating methods -------------------- */
void Solver::updateImmediate(BeliefNode *node, Action const &action, Observation const &observation,
        double deltaTotalQ, long deltaNVisits) {

    // all zero => no update required.
    if (deltaTotalQ == 0 && deltaNVisits == 0) {
        return;
    }

    ActionMapping *actionMap = node->getMapping();
    actionMap->getActionNode(action)->getMapping()->updateVisitCount(observation, deltaNVisits);

    if (actionMap->update(action, deltaNVisits, deltaTotalQ)) {
        addNodeToBackup(node);
    }
}

void Solver::updateEstimate(BeliefNode *node, double deltaTotalQ, long deltaNContinuations) {
    if (node->getParentEntry() == nullptr) {
        return;
    }

    deltaTotalQ += deltaNContinuations * node->getQValue();

    // Apply the discount factor.
    deltaTotalQ *= model_->getDiscountFactor();

    ActionMappingEntry *parentActionEntry = node->getParentActionNode()->getParentEntry();
    if (parentActionEntry->update(0, deltaTotalQ)) {
        addNodeToBackup(node);
    }
}

void Solver::addNodeToBackup(BeliefNode *node) {
    nodesToBackup_[node->getDepth()].insert(node);
}

void Solver::doBackup() {
    while (!nodesToBackup_.empty()) {
        auto firstEntry = nodesToBackup_.cbegin();
        long depth = firstEntry->first;
        for (BeliefNode *node : firstEntry->second) {
            if (depth == 0) {
                node->recalculate();
            } else {
                double oldQValue = node->getQValue();
                node->recalculate();
                double deltaQValue = node->getQValue() - oldQValue;
                long nContinuations = node->getMapping()->getTotalVisitCount()
                        - node->getNumberOfStartingSequences();
                double deltaTotalQ = model_->getDiscountFactor() * nContinuations * deltaQValue;

                ActionMappingEntry *parentActionEntry =
                        node->getParentActionNode()->getParentEntry();
                if (parentActionEntry->update(0, deltaTotalQ)) {
                    addNodeToBackup(parentActionEntry->getMapping()->getOwner());
                }
            }
        }
        nodesToBackup_.erase(firstEntry);
    }
}

} /* namespace solver */
