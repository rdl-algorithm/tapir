#include "DiscreteActionMap.hpp"

#include <cmath>                        // for log, sqrt

#include <iostream>
#include <memory>
#include <utility>
#include <unordered_map>
#include <vector>

#include "Action.hpp"
#include "ActionNode.hpp"
#include "Model.hpp"

#include "global.hpp"

namespace solver {

DiscreteActionMap::DiscreteActionMap(Model *model) :
        model_(model),
        actions_(),
        actionsToTry_(),
        bestAction_(nullptr),
        bestMeanQValue_(0),
        actionMap_() {
}

DiscreteActionMap::DiscreteActionMap(Model *model,
        std::vector<std::unique_ptr<Action>> actions) :
        DiscreteActionMap(model) {
    actions_ = std::move(actions);
    for (std::unique_ptr<Action> &action : actions_) {
        actionsToTry_.add(action.get());
    }
}

DiscreteActionMap::~DiscreteActionMap() {
}

ActionNode *DiscreteActionMap::getActionNode(const Action &action) const {
    try {
        return actionMap_.at(&action).get();
    } catch (const std::out_of_range &oor) {
        return nullptr;
    }
}

ActionNode *DiscreteActionMap::createActionNode(Action const &action) {
    std::unique_ptr<Action> myAction(action.copy());
    std::unique_ptr<ActionNode> node(std::make_unique<ActionNode>(
            model_->createObservationMapping(), &action));
    ActionNode *nodePtr = node.get();
    actionMap_.emplace(myAction.get(), std::move(node));
    actions_.push_back(std::move(myAction));
    return nodePtr;
}

long DiscreteActionMap::size() const {
    return actionMap_.size();
}

bool DiscreteActionMap::hasActionToTry() const {
    return actionsToTry_.size() > 0;
}

std::unique_ptr<Action> DiscreteActionMap::getNextActionToTry(
        RandomGenerator *randGen) {
    long index = std::uniform_int_distribution<long>(
            0, actionsToTry_.size()-1)(*randGen);
    Action const *action = actionsToTry_.get(index);
    actionsToTry_.remove(action);
    return action->copy();
}

std::unique_ptr<Action> DiscreteActionMap::getSearchAction(
        double exploreCoefficient) {
    ActionMap::iterator actionIter = actionMap_.begin();
    double maxVal = (actionIter->second->getMeanQValue() + exploreCoefficient
                     *std::sqrt(std::log(size())
                             / actionIter->second->getNParticles()));
    Action const *bestAction = actionIter->first;
    actionIter++;
    for (; actionIter != actionMap_.end(); actionIter++) {
        double tmpVal = (actionIter->second->getMeanQValue() + exploreCoefficient
             * std::sqrt(
                     std::log(size())
                     / actionIter->second->getNParticles()));
        if (maxVal < tmpVal) {
            maxVal = tmpVal;
            bestAction = actionIter->first;
        }
    }
    return bestAction->copy();
}

void DiscreteActionMap::updateBestValue() {
    if (size() == 0) {
        bestAction_ = nullptr;
        bestMeanQValue_ = 0;
        std::cerr << "No children - could not update Q-value!" << std::endl;
        return;
    }

    ActionMap::iterator actionIter = actionMap_.begin();
    bestMeanQValue_ = actionIter->second->getMeanQValue();
    Action const *bestAction = actionIter->first;
    actionIter++;
    for (; actionIter != actionMap_.end(); actionIter++) {
        if (bestMeanQValue_ < actionIter->second->getMeanQValue()) {
            bestMeanQValue_ = actionIter->second->getMeanQValue();
            bestAction = actionIter->first;
        }
    }
    bestAction_ = bestAction->copy();
}

std::unique_ptr<Action> DiscreteActionMap::getBestAction() const {
    return bestAction_->copy();
}

double DiscreteActionMap::getBestMeanQValue() const {
    return bestMeanQValue_;
}

} /* namespace solver */
