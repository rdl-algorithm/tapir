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
    ActionNode(nullptr) {
}

ActionNode::ActionNode(std::unique_ptr<ObservationMapping> mapping) :
    observationMap_(std::move(mapping)) {
}

// Default destructor
ActionNode::~ActionNode() {
}

ObservationMapping *ActionNode::getMapping() {
    return observationMap_.get();
}

BeliefNode *ActionNode::getChild(Observation const &obs) const {
    return observationMap_->getBelief(obs);
}

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
