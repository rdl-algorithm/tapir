#include "solver/ActionNode.hpp"

#include <memory>                       // for unique_ptr
#include <utility>                      // for make_pair, move, pair
#include <vector>                       // for vector

#include "global.hpp"                     // for make_unique

#include "solver/BeliefNode.hpp"
#include "solver/Solver.hpp"

#include "solver/belief-estimators/estimators.hpp"

#include "solver/abstract-problem/Action.hpp"                   // for Action
#include "solver/abstract-problem/Observation.hpp"              // for Observation

#include "solver/mappings/actions/ActionPool.hpp"       // for ActionPool
#include "solver/mappings/observations/ObservationMapping.hpp"       // for ObservationMapping

namespace solver {
ActionNode::ActionNode() :
        parentEntry_(nullptr),
        observationMap_(nullptr) {
}

ActionNode::ActionNode(ActionMappingEntry *parentEntry) :
        parentEntry_(parentEntry),
        observationMap_(nullptr) {
}

// Default destructor
ActionNode::~ActionNode() {
}

/* -------------------- Tree-related getters  ---------------------- */
ObservationMapping *ActionNode::getMapping() const {
    return observationMap_.get();
}
ActionMappingEntry *ActionNode::getParentEntry() const {
    return parentEntry_;
}
BeliefNode *ActionNode::getParentBelief() const {
    return parentEntry_->getMapping()->getOwner();
}
BeliefNode *ActionNode::getChild(Observation const &obs) const {
    return observationMap_->getBelief(obs);
}


/* ============================ PRIVATE ============================ */


/* -------------------- Tree-related setters  ---------------------- */
void ActionNode::setMapping(std::unique_ptr<ObservationMapping> mapping) {
    observationMap_ = std::move(mapping);
    observationMap_->setOwner(this);
}

/* -------------------- Tree-related methods  ---------------------- */
std::pair<BeliefNode *, bool> ActionNode::createOrGetChild(Solver */*solver*/,
        Observation const &obs) {
    BeliefNode *childNode = getChild(obs);
    bool added = false;
    if (childNode == nullptr) {
        childNode = observationMap_->createBelief(obs);
        added = true;
    }
    return std::make_pair(childNode, added);
}
} /* namespace solver */
