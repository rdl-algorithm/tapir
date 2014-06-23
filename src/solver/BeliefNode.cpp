#include "solver/BeliefNode.hpp"

#include <map>                          // for _Rb_tree_iterator, map<>::iterator, map
#include <memory>                       // for unique_ptr
#include <random>                       // for uniform_int_distribution
#include <set>
#include <tuple>                        // for tie, tuple
#include <utility>                      // for pair, make_pair, move

#include "global.hpp"                     // for RandomGenerator, make_unique
#include "RandomAccessSet.hpp"

#include "solver/cached_values.hpp"

#include "solver/ActionNode.hpp"               // for ActionNode
#include "solver/HistoryEntry.hpp"             // for HistoryEntry
#include "solver/Solver.hpp"                   // for Solver

#include "solver/abstract-problem/Action.hpp"                   // for Action
#include "solver/abstract-problem/HistoricalData.hpp"
#include "solver/abstract-problem/Observation.hpp"              // for Observation
#include "solver/abstract-problem/State.hpp"                    // for State

#include "solver/search/action-choosers/choosers.hpp"

#include "solver/mappings/actions/ActionMapping.hpp"
#include "solver/mappings/observations/ObservationMapping.hpp"
#include "solver/mappings/observations/ObservationPool.hpp"

namespace solver {
BeliefNode::BeliefNode() :
            BeliefNode(-1, nullptr) {
}
BeliefNode::BeliefNode(ObservationMappingEntry *parentEntry) :
            BeliefNode(-1, parentEntry) {
}

BeliefNode::BeliefNode(long id, ObservationMappingEntry *parentEntry) :
            id_(id),
            depth_(-1),
            parentEntry_(parentEntry),
            data_(nullptr),
            particles_(),
            nStartingSequences_(0),
            tLastChange_(-1),
            actionMap_(nullptr),
            cachedValues_(),
            qEstimator_(nullptr) {
    if (parentEntry_ == nullptr) {
        depth_ = 0;
    } else {
        depth_ = getParentBelief()->getDepth() + 1;
    }
}

// Do-nothing destructor
BeliefNode::~BeliefNode() {
}

/* ----------------- Useful calculations ------------------- */
double BeliefNode::distL1Independent(BeliefNode *b) const {
    double dist = 0.0;
    for (HistoryEntry *entry1 : particles_) {
        for (HistoryEntry *entry2 : b->particles_) {
            dist += entry1->getState()->distanceTo(*entry2->getState());
        }
    }
    double averageDist = dist / (getNumberOfParticles() * b->getNumberOfParticles());
    if (averageDist < 0) {
        debug::show_message("ERROR: Distance < 0 between beliefs.");
    } else if (averageDist == 0) {
        debug::show_message("NOTE: Identical belief nodes found!");
    }
    return averageDist;
}

/* -------------------- Simple getters ---------------------- */
long BeliefNode::getId() const {
    return id_;
}
long BeliefNode::getDepth() const {
    return depth_;
}
long BeliefNode::getNumberOfParticles() const {
    return particles_.size();
}
long BeliefNode::getNumberOfStartingSequences() const {
    return nStartingSequences_;
}
std::vector<State const *> BeliefNode::getStates() const {
    std::vector<State const *> states;
    for (HistoryEntry *entry : particles_) {
        states.push_back(entry->getState());
    }
    return states;
}
double BeliefNode::getTimeOfLastChange() const {
    return tLastChange_;
}

/* -------------------- Tree-related getters  ---------------------- */
ActionMapping *BeliefNode::getMapping() const {
    return actionMap_.get();
}
ObservationMappingEntry *BeliefNode::getParentEntry() const {
    return parentEntry_;
}
ActionNode *BeliefNode::getParentActionNode() const {
    if (parentEntry_ == nullptr) {
        return nullptr;
    }
    return parentEntry_->getMapping()->getOwner();
}
HistoricalData *BeliefNode::getHistoricalData() const {
    return data_.get();
}
BeliefNode *BeliefNode::getParentBelief() const {
    if (parentEntry_ == nullptr) {
        return nullptr;
    }
    return getParentActionNode()->getParentEntry()->getMapping()->getOwner();
}
std::unique_ptr<Observation> BeliefNode::getLastObservation() const {
    if (parentEntry_ == nullptr) {
        return nullptr;
    }
    return parentEntry_->getObservation();
}
std::unique_ptr<Action> BeliefNode::getLastAction() const {
    if (parentEntry_ == nullptr) {
        return nullptr;
    }
    return getParentActionNode()->getParentEntry()->getAction();
}
BeliefNode *BeliefNode::getChild(Action const &action, Observation const &obs) const {
    ActionNode *node = actionMap_->getActionNode(action);
    if (node == nullptr) {
        return nullptr;
    }
    return node->getChild(obs);
}


/* -------------------- Simple setters  ---------------------- */
void BeliefNode::updateTimeOfLastChange() {
    // tLastChange_ = abt::clock_ms();
}

/* ----------------- Management of cached values ------------------- */
BaseCachedValue *BeliefNode::addCachedValue(std::unique_ptr<BaseCachedValue> value) {
    BaseCachedValue *rawPtr = value.get();
    cachedValues_[rawPtr] = std::move(value);
    return rawPtr;
}
void BeliefNode::removeCachedValue(BaseCachedValue *value) {
    cachedValues_.erase(value);
}

/* ------------ Q-value calculation and action selection -------------- */
void BeliefNode::setQEstimator(CachedValue<double> *qEstimator) {
    qEstimator_ = qEstimator;
}

/* ------------ Q-value calculation and action selection -------------- */
std::unique_ptr<Action> BeliefNode::getRecommendedAction() const {
    return choosers::max_action(this);
}
double BeliefNode::getQValue() const {
    return qEstimator_->getCache();
}
void BeliefNode::recalculateQValue() {
    qEstimator_->updateCache();
}

/* ============================ PRIVATE ============================ */

/* -------------- Particle management / sampling ---------------- */
void BeliefNode::addParticle(HistoryEntry *newHistEntry) {
    updateTimeOfLastChange();
    particles_.add(newHistEntry);
    if (newHistEntry->getId() == 0) {
        nStartingSequences_++;
    }
}

void BeliefNode::removeParticle(HistoryEntry *histEntry) {
    updateTimeOfLastChange();
    particles_.remove(histEntry);
    if (histEntry->getId() == 0) {
        nStartingSequences_--;
    }
}

/* -------------------- Tree-related setters  ---------------------- */
void BeliefNode::setMapping(std::unique_ptr<ActionMapping> mapping) {
    actionMap_ = std::move(mapping);
}
void BeliefNode::setHistoricalData(std::unique_ptr<HistoricalData> data) {
    data_ = std::move(data);
}

/* -------------------- Tree-related methods  ---------------------- */
std::pair<BeliefNode *, bool> BeliefNode::createOrGetChild(Solver *solver, Action const &action,
        Observation const &obs) {
    ActionNode *actionNode = actionMap_->getActionNode(action);
    if (actionNode == nullptr) {
        actionNode = actionMap_->createActionNode(action);
        actionNode->setMapping(solver->getObservationPool()->createObservationMapping(actionNode));
    }
    return actionNode->createOrGetChild(solver, obs);
}
} /* namespace solver */
