#include "BeliefNode.hpp"

#include <map>                          // for _Rb_tree_iterator, map<>::iterator, map
#include <memory>                       // for unique_ptr
#include <random>                       // for uniform_int_distribution
#include <set>
#include <tuple>                        // for tie, tuple
#include <utility>                      // for pair, make_pair, move

#include "global.hpp"                     // for RandomGenerator, make_unique
#include "RandomAccessSet.hpp"

#include "ActionNode.hpp"               // for ActionNode
#include "HistoryEntry.hpp"             // for HistoryEntry
#include "Solver.hpp"                   // for Solver

#include "abstract-problem/Action.hpp"                   // for Action
#include "abstract-problem/Observation.hpp"              // for Observation
#include "abstract-problem/State.hpp"                    // for State

#include "mappings/ActionMapping.hpp"
#include "mappings/ObservationMapping.hpp"
#include "mappings/ObservationPool.hpp"

#include "search/HistoricalData.hpp"


namespace solver {
BeliefNode::BeliefNode() :
    BeliefNode(nullptr) {
}

BeliefNode::BeliefNode(ObservationMappingEntry *parentEntry) :
        id_(-1),
        parentEntry_(parentEntry),
        data_(nullptr),
        particles_(),
        nStartingSequences_(0),
        tLastChange_(-1),
        actionMap_(nullptr) {
}

// Do-nothing destructor
BeliefNode::~BeliefNode() {
}

/* -------------- Particle management / sampling ---------------- */
void BeliefNode::addParticle(HistoryEntry *newHistEntry) {
    tLastChange_ = abt::clock_ms();
    particles_.add(newHistEntry);
    if (newHistEntry->getId() == 0) {
        nStartingSequences_++;
    }
}

void BeliefNode::removeParticle(HistoryEntry *histEntry) {
    tLastChange_ = abt::clock_ms();
    particles_.remove(histEntry);
    if (histEntry->getId() == 0) {
        nStartingSequences_--;
    }
}

HistoryEntry *BeliefNode::sampleAParticle(RandomGenerator *randGen) const {
    long index = std::uniform_int_distribution<long>(
                                 0, getNumberOfParticles() - 1)(*randGen);
    return particles_.get(index);
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

/* -------------------- Simple setters ---------------------- */
void BeliefNode::setId(long id) {
    id_ = id;
}

/* -------------------- Simple getters ---------------------- */
long BeliefNode::getId() const {
    return id_;
}
std::unique_ptr<Action> BeliefNode::getRecommendedAction() const {
    return actionMap_->getRecommendedAction();
}
double BeliefNode::getQValue() const {
    return actionMap_->getMaxQValue();
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

/* -------------------- Tree-related setters  ---------------------- */
void BeliefNode::setMapping(std::unique_ptr<ActionMapping> mapping) {
    actionMap_ = std::move(mapping);
    actionMap_->setOwner(this);
}
void BeliefNode::setParentEntry(ObservationMappingEntry *entry) {
    parentEntry_ =  entry;
}
void BeliefNode::setHistoricalData(std::unique_ptr<HistoricalData> data) {
    data_ = std::move(data);
}

/* -------------------- Tree-related getters  ---------------------- */
ActionMapping *BeliefNode::getMapping() const {
    return actionMap_.get();
}
ObservationMappingEntry *BeliefNode::getParentEntry() const{
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

/* -------------------- Tree-related methods  ---------------------- */
std::pair<BeliefNode *, bool> BeliefNode::createOrGetChild(Solver *solver,
        Action const &action, Observation const &obs) {
    ActionNode *actionNode = actionMap_->getActionNode(action);
    if (actionNode == nullptr) {
        actionNode = actionMap_->createActionNode(action);
        actionNode->setMapping(solver->getObservationPool()->createObservationMapping());
    }
    return actionNode->createOrGetChild(solver, obs);
}
} /* namespace solver */
