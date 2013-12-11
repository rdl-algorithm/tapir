#include "ActionNode.hpp"

#include <memory>                       // for unique_ptr
#include <utility>                      // for make_pair, move, pair
#include <vector>                       // for vector

#include "defs.hpp"                     // for make_unique

#include "Action.hpp"                   // for Action
#include "Observation.hpp"              // for Observation
#include "ObservationEdge.hpp"          // for ObservationEdge

ActionNode::ActionNode() :
    ActionNode(-1) {
}

ActionNode::ActionNode(Action const &action) :
    action(action),
    nParticles(0),
    totalQValue(0),
    meanQValue(0),
    obsChildren() {
}

void ActionNode::updateQValue(double increase) {
    totalQValue += increase;
    if (nParticles > 0) {
        meanQValue = totalQValue / nParticles;
    } else {
        meanQValue = 0;
    }
}

// Default destructor
ActionNode::~ActionNode() {
}

void ActionNode::updateQValue(double oldValue, double newValue,
        bool reduceParticles) {
    if (reduceParticles) {
        nParticles--;
    }
    updateQValue(newValue - oldValue);
}

std::pair<BeliefNode *, bool> ActionNode::addChild(Observation const &obs) {
    BeliefNode *beliefChild = getBeliefChild(obs);
    bool added = false;
    if (beliefChild == nullptr) {
        std::unique_ptr<ObservationEdge> newEdge =
            std::make_unique<ObservationEdge>(obs);
        beliefChild = newEdge->getBeliefChild();
        obsChildren.push_back(std::move(newEdge));
        added = true;
    }
    nParticles++;
    return std::make_pair(beliefChild, added);
}

BeliefNode *ActionNode::getBeliefChild(Observation const &obs) {
    for (std::unique_ptr<ObservationEdge> &child : obsChildren) {
        if (child->obsEquals(obs)) {
            return child->getBeliefChild();
        }
    }
    return nullptr;
}