/** file: Solver.cpp
 *
 * Contains the implementation of the Solver class.
 */
#include "solver/Solver.hpp"

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

#include "solver/abstract-problem/Action.hpp"                   // for Action
#include "solver/abstract-problem/Model.hpp"                    // for Model::StepResult, Model
#include "solver/abstract-problem/ModelChange.hpp"                    // for Model::StepResult, Model
#include "solver/abstract-problem/Observation.hpp"              // for Observation
#include "solver/abstract-problem/State.hpp"                    // for State, operator<<

#include "solver/belief-estimators/estimators.hpp"

#include "solver/changes/ChangeFlags.hpp"               // for ChangeFlags, ChangeFlags::UNCHANGED, ChangeFlags::ADDOBSERVATION, ChangeFlags::ADDOBSTACLE, ChangeFlags::ADDSTATE, ChangeFlags::DELSTATE, ChangeFlags::REWARD, ChangeFlags::TRANSITION
#include "solver/changes/HistoryCorrector.hpp"

#include "solver/mappings/actions/ActionMapping.hpp"
#include "solver/mappings/actions/ActionPool.hpp"
#include "solver/mappings/observations/ObservationMapping.hpp"
#include "solver/mappings/observations/ObservationPool.hpp"

#include "solver/search/SearchStatus.hpp"
#include "solver/search/search_interface.hpp"

#include "solver/serialization/Serializer.hpp"               // for Serializer

#include "solver/indexing/RTree.hpp"
#include "solver/indexing/SpatialIndexVisitor.hpp"

#include "solver/ActionNode.hpp"               // for BeliefNode, BeliefNode::startTime
#include "solver/BeliefNode.hpp"               // for BeliefNode, BeliefNode::startTime
#include "solver/BeliefTree.hpp"               // for BeliefTree
#include "solver/Histories.hpp"                // for Histories
#include "solver/HistoryEntry.hpp"             // for HistoryEntry
#include "solver/HistorySequence.hpp"          // for HistorySequence
#include "solver/StateInfo.hpp"                // for StateInfo
#include "solver/StatePool.hpp"                // for StatePool

using std::cout;
using std::endl;

namespace solver {
Solver::Solver(std::unique_ptr<Model> model) :
            model_(std::move(model)),
            options_(model_->getOptions()),
            serializer_(model_->createSerializer(this)),
            statePool_(nullptr),
            histories_(nullptr),
            policy_(nullptr),
            actionPool_(nullptr),
            observationPool_(nullptr),
            historyCorrector_(nullptr),
            searchStrategy_(nullptr),
            estimationStrategy_(nullptr),
            nodesToBackup_(),
            changeRoot_(nullptr),
            isAffectedMap_() {
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
Options const *Solver::getOptions() const {
    return options_;
}
ActionPool *Solver::getActionPool() const {
    return actionPool_.get();
}
ObservationPool *Solver::getObservationPool() const {
    return observationPool_.get();
}
EstimationStrategy *Solver::getEstimationStrategy() const {
    return estimationStrategy_.get();
}
Serializer *Solver::getSerializer() const {
    return serializer_.get();
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

/* ------------------- Policy mutators ------------------- */
void Solver::improvePolicy(BeliefNode *startNode, long numberOfHistories, long maximumDepth,
        double timeout) {
    double startTime = abt::clock_ms();
    if (numberOfHistories < 0) {
        numberOfHistories = options_->historiesPerStep;
    }
    if (maximumDepth < 0) {
        maximumDepth = options_->maximumDepth;
    }
    if (timeout < 0) {
        timeout = options_->stepTimeout;
    }

    // No timeout => end at t=inf
    if (timeout == 0) {
        // 0 => no timeout (infinity)
        timeout = std::numeric_limits<double>::infinity();
    }

    // Retrieve the sampling function to use.
    std::function<StateInfo *()> sampler = getStateSampler(startNode);
    if (sampler == nullptr) {
        return;
    }

    // Null start node => use the root.
    if (startNode == nullptr) {
        startNode = policy_->getRoot();
    }

    long actualNumHistories = multipleSearches(startNode,  sampler, maximumDepth, numberOfHistories,
            startTime + timeout);
    double totalTimeTaken = abt::clock_ms() - startTime;
    if (options_->hasVerboseOutput) {
        cout << actualNumHistories << " histories in " << totalTimeTaken << "ms." << endl;
    }
}

BeliefNode *Solver::replenishChild(BeliefNode *currNode, Action const &action,
        Observation const &obs, long minParticleCount) {
    if (minParticleCount < 0) {
        minParticleCount = options_->minParticleCount;
    }
    BeliefNode *nextNode = policy_->createOrGetChild(currNode, action, obs);
    long particleCount = nextNode->getNumberOfParticles();
    long deficit = minParticleCount - particleCount;
    if (deficit <= 0) {
        return nextNode;
    }

    if (options_->hasVerboseOutput) {
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
        HistoryEntry *histEntry = histSeq->addEntry();
        histEntry->registerState(stateInfo);
        histEntry->registerNode(nextNode);
    }
    if (options_->hasVerboseOutput) {
        cout << "Done" << std::endl;
    }
    return nextNode;
}

/* ------------------- Change handling methods ------------------- */
BeliefNode *Solver::getChangeRoot() const {
    return changeRoot_;
}

void Solver::setChangeRoot(BeliefNode *changeRoot) {
    if (changeRoot_ != changeRoot) {
        changeRoot_ = changeRoot;
        isAffectedMap_.clear();
    }
}

bool Solver::isAffected(BeliefNode const *node) {
    if (changeRoot_ == nullptr) {
        // No change root => all nodes are affected.
        return true;
    }
    if (node == changeRoot_) {
        // Same node => must be affected.
        return true;
    }

    long rootDepth = changeRoot_->getDepth();
    if (node->getDepth() <= rootDepth) {
        // Not below the root and not the root => can't be affected.
        return false;
    }

    // If it's deeper, look it up in the mapping.
    std::unordered_map<BeliefNode const *, bool>::iterator it = isAffectedMap_.find(node);
    if (it != isAffectedMap_.end()) {
        return it->second;
    }

    // We have to traverse the tree to see if it's a descendant.
    std::vector<BeliefNode const *> ancestors;
    bool isDescendedFromChangeRoot = false;
    while (true) {
        ancestors.push_back(node);
        node = node->getParentBelief();
        // If we hit the root, we must be descended from it.
        if (node == changeRoot_) {
            isDescendedFromChangeRoot = true;
            break;
        }
        // If we reach the same depth without hitting the root, we aren't descended from it.
        if (node->getDepth() == rootDepth) {
            break;
        }

        it = isAffectedMap_.find(node);
        // If we hit a node for which we know whether it's descended, we copy its state.
        if (it != isAffectedMap_.end()) {
            isDescendedFromChangeRoot = it->second;
            break;
        }
    }

    // Add ancestors to the mapping for efficient lookup later on.
    for (BeliefNode const *ancestor : ancestors) {
        isAffectedMap_[ancestor] = isDescendedFromChangeRoot;
    }

    return isDescendedFromChangeRoot;
}

void Solver::applyChanges() {
    std::unordered_set<HistorySequence *> affectedSequences;
    for (StateInfo *stateInfo : statePool_->getAffectedStates()) {
        if (changes::has_flags(stateInfo->changeFlags_, ChangeFlags::DELETED)) {
            // Deletion implies the prior transition must be invalid.
            stateInfo->changeFlags_ |= ChangeFlags::TRANSITION_BEFORE;
        }

        for (HistoryEntry *entry : stateInfo->usedInHistoryEntries_) {
            BeliefNode *node = entry->getAssociatedBeliefNode();

            // If this node isn't affected, ignore it.
            if (!isAffected(node)) {
                continue;
            }

            HistorySequence *sequence = entry->owningSequence_;
            HistoryEntry::IdType entryId = entry->entryId_;

            // True iff any changes have been applied to this sequence.
            bool hasChanged = false;
            ChangeFlags allFlags = stateInfo->changeFlags_;

            if (node == changeRoot_) {
                // Any flags applying to previous states must be ignored.
                allFlags &= ~ChangeFlags::TRANSITION_BEFORE;
                allFlags &= ~ChangeFlags::OBSERVATION_BEFORE;
                allFlags &= ~ChangeFlags::HEURISTIC;

                // If a state at the change root is deleted, just delete the whole sequence.
                if (changes::has_flags(allFlags, ChangeFlags::DELETED)) {
                    hasChanged = true;
                    sequence->setChangeFlags(0, ChangeFlags::DELETED);
                }
            }

            // TRANSITION_BEFORE => TRANSITION for previous entry.
            if (changes::has_flags(allFlags, ChangeFlags::TRANSITION_BEFORE)) {
                allFlags &= ~ChangeFlags::TRANSITION_BEFORE;
                if (entryId > 0) {
                    hasChanged = true;
                    sequence->setChangeFlags(entryId - 1, ChangeFlags::TRANSITION);
                }
            }

            // OBSERVATION_BEFORE => OBSERVATION for previous entry.
            if (changes::has_flags(allFlags, ChangeFlags::OBSERVATION_BEFORE)) {
                allFlags &= ~ChangeFlags::OBSERVATION_BEFORE;
                if (entryId > 0) {
                    hasChanged = true;
                    sequence->setChangeFlags(entryId - 1, ChangeFlags::OBSERVATION);
                }
            }

            // Heuristic changes only apply if it's the last entry in the sequence.
            if(changes::has_flags(allFlags, ChangeFlags::HEURISTIC)) {
                allFlags &= ~ChangeFlags::HEURISTIC;
                if (entry->action_ == nullptr) {
                    hasChanged = true;
                    sequence->setChangeFlags(entryId, ChangeFlags::HEURISTIC);
                }
            }

            // Apply any remaining flags, if any.
            if (allFlags != ChangeFlags::UNCHANGED) {
                hasChanged = true;
                sequence->setChangeFlags(entryId, allFlags);
            }
            if (hasChanged) {
                // The sequence is only affected if one of the changes went through.
                affectedSequences.insert(sequence);
            }
        }
    }

    long numAffected = affectedSequences.size();

    if (options_->hasVerboseOutput) {
        cout << "Must revise " << numAffected << " of ";
        cout << histories_->getNumberOfSequences() << " histories!" << endl;
    }

    // Delete and remove any sequences where the first state has been deleted.
    std::unordered_set<HistorySequence *>::iterator it = affectedSequences.begin();
    while (it != affectedSequences.end()) {
        HistorySequence *sequence = *it;
        if (changes::has_flags(sequence->getFirstEntry()->changeFlags_, ChangeFlags::DELETED)) {
            it = affectedSequences.erase(it);
            // Now we undo the sequence, and delete it entirely.
            updateSequence(sequence, -1);
            histories_->deleteSequence(sequence);
        } else {
            it++;
        }
    }

    if (options_->hasVerboseOutput) {
        cout << "Deleted " << numAffected - affectedSequences.size() << " histories!" << endl;
    }

    // Revise all of the histories.
    historyCorrector_->reviseHistories(affectedSequences);

    if (options_->hasVerboseOutput) {
        cout << "Revision complete. Backing up..." << endl;
    }

    // Finish the deferred backup so that algorithms requiring backed-up values will work properly.
    doBackup();

    // Reset the set of affected states in the pool.
    statePool_->resetAffectedStates();

    // Now we extend sequences with illegal actions.
    if (options_->hasVerboseOutput) {
        cout << "Using search on " << affectedSequences.size() << " histories!" << endl;
    }

    // Extend and backup each sequence.
    for (HistorySequence *sequence : affectedSequences) {
        searchStrategy_->extendAndBackup(sequence, options_->maximumDepth);
    }

    // Backup all the way to the root to keep the tree consistent.
    doBackup();
}

/* ------------------ Display methods  ------------------- */
void Solver::printBelief(BeliefNode *belief, std::ostream &os) {
    os << belief->getCachedValue();
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

void Solver::printTree(std::ostream &/*os*/) {
}

/* -------------- Management of deferred backpropagation. --------------- */
bool Solver::isBackedUp() const {
    return nodesToBackup_.empty();
}

void Solver::doBackup() {
    while (!nodesToBackup_.empty()) {
        auto firstEntry = nodesToBackup_.cbegin();
        long depth = firstEntry->first;
        for (BeliefNode *node : firstEntry->second) {
            if (depth == 0) {
                node->recalculateValue();
            } else {
                double oldQValue = node->getCachedValue();
                node->recalculateValue();
                double deltaQValue = node->getCachedValue() - oldQValue;
                long nContinuations = node->getMapping()->getTotalVisitCount()
                        - node->getNumberOfStartingSequences();
                double deltaTotalQ = options_->discountFactor * nContinuations * deltaQValue;

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


/* ------------------ Methods to update the q-values in the tree. ------------------- */
void Solver::updateSequence(HistorySequence *sequence, int sgn, long firstEntryId,
        bool propagateQChanges) {
    // Cannot update sequences of length <= 1.
    if (sequence->getLength() <= 1) {
        return;
    }

    double discountFactor = options_->discountFactor;


    // Traverse the sequence in reverse.
    auto it = sequence->entrySequence_.crbegin();

    // The last entry is used only for the heuristic estimate.
    double deltaTotalQ = (*it)->immediateReward_;
    it++;
    BeliefNode *node;
    while (true) {
        // Apply discount and add the immediate reward.
        deltaTotalQ = deltaTotalQ * discountFactor + (*it)->immediateReward_;
        node = (*it)->getAssociatedBeliefNode();
        ActionMapping *mapping = node->getMapping();
        ActionMappingEntry *entry = mapping->getEntry(*(*it)->getAction());
        // Update the action value and visit count.
        entry->update(sgn, sgn * deltaTotalQ);
        // Update the observation visit count.
        entry->getActionNode()->getMapping()->updateVisitCount(*(*it)->getObservation(), sgn);


        // If we've gone past the source node, we don't need to update further.
        // Backpropagation may need to go further, but we can simply defer it.
        it++;
        if (it == sequence->entrySequence_.crend() || (*it)->entryId_ < firstEntryId) {
            addNodeToBackup(node);
            break;
        }

        double oldQ = node->getCachedValue();
        deltaTotalQ = oldQ;
        if (propagateQChanges) {
            // Backpropagate the change in the q-value of the node.
            node->recalculateValue();
            double newQ = node->getCachedValue();
            long nContinuations = mapping->getTotalVisitCount() - node->getNumberOfStartingSequences();
            deltaTotalQ += (newQ - oldQ) * nContinuations;
        } else {
            // If we haven't backpropagated the change, we need to do it later.
            addNodeToBackup(node);
        }
    }
}

void Solver::updateEstimate(BeliefNode *node, double deltaTotalQ, long deltaNContinuations) {
    if (node->getParentEntry() == nullptr) {
        return;
    }

    deltaTotalQ += deltaNContinuations * node->getCachedValue();

    // Apply the discount factor.
    deltaTotalQ *= options_->discountFactor;

    ActionMappingEntry *parentActionEntry = node->getParentActionNode()->getParentEntry();
    if (parentActionEntry->update(0, deltaTotalQ)) {
        addNodeToBackup(parentActionEntry->getMapping()->getOwner());
    }
}

void Solver::updateImmediate(BeliefNode *node, Action const &action, Observation const &observation,
        double deltaTotalQ, long deltaNVisits) {

    // all zero => no update required.
    if (deltaTotalQ == 0 && deltaNVisits == 0) {
        return;
    }

    // Retrieve the associated ActionMappingEntry.
    ActionMappingEntry *entry = node->getMapping()->getEntry(action);

    // Update the visit count for the observation.
    entry->getActionNode()->getMapping()->updateVisitCount(observation, deltaNVisits);

    // Update the action.
    if (entry->update(deltaNVisits, deltaTotalQ)) {
        addNodeToBackup(node);
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
    searchStrategy_ = model_->createSearchStrategy(this);
    estimationStrategy_ = model_->createEstimationStrategy(this);
}

/* ------------------ Episode sampling methods ------------------- */
std::function<StateInfo *()> Solver::getStateSampler(BeliefNode *node) {
    // Nullptr => sample initial sates from the model.
    if (node == nullptr) {
        return [this]() {
            return statePool_->createOrGetInfo(*model_->sampleAnInitState());
        };
    }

    // Filter out any terminal states.
    std::vector<StateInfo *> nonTerminalStates;
    for (long index = 0; index < node->getNumberOfParticles(); index++) {
        HistoryEntry *entry = node->particles_.get(index);
        if (!model_->isTerminal(*entry->getState())) {
            nonTerminalStates.push_back(entry->stateInfo_);
        }
    }

    // No non-terminal states => return an empty function.
    if (nonTerminalStates.empty()) {
        debug::show_message("ERROR: No non-terminal particles in the current node!");
        return std::function<StateInfo *()>();
    }

    RandomGenerator *randGen = model_->getRandomGenerator();
    return [randGen, nonTerminalStates]() {
        long index = std::uniform_int_distribution<long>(0, nonTerminalStates.size() - 1)(*randGen);
        return nonTerminalStates[index];
    };
}

long Solver::multipleSearches(BeliefNode *startNode, std::function<StateInfo *()> sampler,
        long maximumDepth, long maxNumSearches, double endTime) {
    bool hasTimeout = true;
    if (endTime == std::numeric_limits<double>::infinity()) {
        hasTimeout = false;
    }

    long numSearches = 0;
    while (true) {
        // If we've done enough searches, stop searching.
        if (maxNumSearches != 0 && numSearches >= maxNumSearches) {
            break;
        }
        // If we've gone past the termination time, stop searching.
        if (hasTimeout && abt::clock_ms() >= endTime) {
            break;
        }
        singleSearch(startNode, sampler(), maximumDepth);
        numSearches++;
    }

    // Backup all the way back to the root of the tree to maintain consistency.
    doBackup();

    return numSearches;
}

void Solver::singleSearch(BeliefNode *startNode, StateInfo *startStateInfo, long maximumDepth) {
    if (maximumDepth < 0) {
        maximumDepth = options_->maximumDepth;
    }

    HistorySequence *sequence = histories_->createSequence();

    HistoryEntry *firstEntry = sequence->addEntry();
    firstEntry->registerState(startStateInfo);
    firstEntry->registerNode(startNode);

    continueSearch(sequence, maximumDepth);
}

void Solver::continueSearch(HistorySequence *sequence, long maximumDepth) {
    if (maximumDepth < 0) {
        maximumDepth = options_->maximumDepth;
    }

    // Extend and backup sequence.
    searchStrategy_->extendAndBackup(sequence, maximumDepth);
}

/* ------------------ Private deferred backup methods. ------------------- */
void Solver::addNodeToBackup(BeliefNode *node) {
    nodesToBackup_[node->getDepth()].insert(node);
}

void Solver::removeNodeToBackup(BeliefNode *node) {
    auto nodeSet = nodesToBackup_[node->getDepth()];
    nodeSet.erase(nodeSet.find(node));
}
} /* namespace solver */
