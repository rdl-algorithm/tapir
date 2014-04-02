#include "ActionNode.hpp"

#include <memory>                       // for unique_ptr
#include <utility>                      // for make_pair, move, pair
#include <vector>                       // for vector

#include "global.hpp"                     // for make_unique

#include "BeliefNode.hpp"

#include "abstract-problem/Action.hpp"                   // for Action
#include "abstract-problem/Observation.hpp"              // for Observation

#include "mappings/ObservationMapping.hpp"       // for ObservationMapping

namespace solver {
ActionNode::ActionNode() :
        parentEntry_(nullptr),
        observationMap_(nullptr) {
}

// Default destructor
ActionNode::~ActionNode() {
}

/* -------------------- Tree-related setters  ---------------------- */
void ActionNode::setMapping(std::unique_ptr<ObservationMapping> mapping) {
    observationMap_ = std::move(mapping);
    observationMap_->setOwner(this);
}
void ActionNode::setParentEntry(ActionMappingEntry *entry) {
    parentEntry_ =  entry;
}

/* -------------------- Tree-related getters  ---------------------- */
ObservationMapping *ActionNode::getMapping() const {
    return observationMap_.get();
}
ActionMappingEntry *ActionNode::getParentEntry() const {
    return parentEntry_;
}
BeliefNode *ActionNode::getChild(Observation const &obs) const {
    return observationMap_->getBelief(obs);
}

/* -------------------- Tree-related methods  ---------------------- */
std::pair<BeliefNode *, bool> ActionNode::createOrGetChild(Observation const &obs) {
    BeliefNode *beliefChild = getChild(obs);
    bool added = false;
    if (beliefChild == nullptr) {
        beliefChild = observationMap_->createBelief(obs);
        added = true;
    }
    return std::make_pair(beliefChild, added);
}
} /* namespace solver */
