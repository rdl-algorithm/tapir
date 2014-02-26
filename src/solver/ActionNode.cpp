#include "ActionNode.hpp"

#include <memory>                       // for unique_ptr
#include <utility>                      // for make_pair, move, pair
#include <vector>                       // for vector

#include "global.hpp"                     // for make_unique

#include "geometry/Action.hpp"                   // for Action
#include "geometry/Observation.hpp"              // for Observation
#include "mappings/ObservationMapping.hpp"       // for ObservationMapping

namespace solver {
ActionNode::ActionNode() :
    ActionNode(nullptr, nullptr) {
}

ActionNode::ActionNode(std::unique_ptr<ObservationMapping> mapping,
        Action const *action) :
    action_((action == nullptr) ? nullptr : action->copy()),
    nParticles_(0),
    totalQValue_(0),
    meanQValue_(0),
    obsMap_(std::move(mapping)) {
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

long ActionNode::getNParticles() const {
    return nParticles_;
}

double ActionNode::getTotalQValue () const {
    return totalQValue_;
}

double ActionNode::getMeanQValue () const {
    return meanQValue_;
}

ObservationMapping *ActionNode::getMapping() {
    return obsMap_.get();
}

BeliefNode *ActionNode::getChild(Observation const &obs) const {
    return obsMap_->getBelief(obs);
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

Action const *ActionNode::getAction() const {
    return action_.get();
}
} /* namespace solver */
