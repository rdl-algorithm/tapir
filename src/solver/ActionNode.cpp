#include "ActionNode.hpp"

#include <memory>                       // for unique_ptr
#include <utility>                      // for make_pair, move, pair
#include <vector>                       // for vector

#include "global.hpp"                     // for make_unique

#include "Action.hpp"                   // for Action
#include "Observation.hpp"              // for Observation
#include "SimpleObsMap.hpp"             // for SimpleObsMap

namespace solver {
ActionNode::ActionNode() :
    ActionNode(-1) {
}

ActionNode::ActionNode(Action const &action) :
    action_(action),
    nParticles_(0),
    totalQValue_(0),
    meanQValue_(0),
    obsMap_(std::make_unique<SimpleObsMap>()) {
}

// Default destructor
ActionNode::~ActionNode() {
}

void ActionNode::updateQValue(double increase) {
    totalQValue_ += increase;
    if (nParticles_ > 0) {
        meanQValue_ = totalQValue_ / nParticles_;
    } else {
        meanQValue_ = 0;
    }
}

void ActionNode::updateQValue(double increase,
        long deltaNParticles) {
    nParticles_ += deltaNParticles;
    updateQValue(increase);
}

std::pair<BeliefNode *, bool> ActionNode::createOrGetChild(Observation const &obs) {
    BeliefNode *beliefChild = getChild(obs);
    bool added = false;
    if (beliefChild == nullptr) {
        beliefChild = obsMap_->createBelief(obs);
        added = true;
    }
    return std::make_pair(beliefChild, added);
}

BeliefNode *ActionNode::getChild(Observation const &obs) const {
    return obsMap_->getBelief(obs);
}
} /* namespace solver */
