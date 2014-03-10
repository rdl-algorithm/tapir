#include "enumerated_actions.hpp"

#include <algorithm>
#include <cmath>

#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"
#include "solver/abstract-problem/Model.hpp"

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/EnumeratedPoint.hpp"

#include "ActionPool.hpp"
#include "ActionMapping.hpp"
#include "ObservationPool.hpp"

namespace solver {
/* ------------------- ModelWithEnumeratedActions ------------------- */
std::unique_ptr<ActionPool>
    ModelWithEnumeratedActions::createActionPool() {
    return std::make_unique<EnumeratedActionPool>(getRandomGenerator(),
            getUcbExploreCoefficient(),
            getAllActionsInOrder());
}

/* --------------------- EnumeratedActionPool --------------------- */
EnumeratedActionPool::EnumeratedActionPool(RandomGenerator *randGen,
        double ucbExplorationCoefficient,
        std::vector<std::unique_ptr<EnumeratedPoint>> actions) :
    randGen_(randGen),
    ucbExplorationCoefficient_(ucbExplorationCoefficient),
    allActions_(std::move(actions)) {
}

std::unique_ptr<ActionMapping>
    EnumeratedActionPool::createActionMapping() {
    return std::make_unique<EnumeratedActionMap>(observationPool_,
            allActions_);
}

/* ---------------------- EnumeratedActionMap ---------------------- */
EnumeratedActionMap::EnumeratedActionMap(ObservationPool *observationPool,
        double ucbExplorationCoefficient,
        std::vector<std::unique_ptr<EnumeratedPoint>> const &allActions) :
                observationPool_(observationPool),
                ucbExplorationCoefficient_(ucbExplorationCoefficient),
                allActions_(allActions),
                entries_(allActions_.size()),
                nChildren_(0),
                bestAction_(nullptr),
                bestMeanQValue_(-std::numeric_limits<double>::infinity()) {
}

ActionNode* EnumeratedActionMap::getActionNode(Action const &action) const {
    long code = static_cast<EnumeratedPoint const &>(action).getCode();
    return entries_[code]->getActionNode();
}

ActionNode* EnumeratedActionMap::createActionNode(Action const &action) {
    long code = static_cast<EnumeratedPoint const &>(action).getCode();
    entries_[code] = std::make_unique<EnumeratedActionMapEntry>(
            action, std::make_unique<ActionNode>(
                    observationPool_->createObservationMapping()));
    nChildren_++;
    return entries_[code]->getActionNode();
}

std::vector<ActionMappingEntry const *> EnumeratedActionMap::getChildEntries() const {
    std::vector<ActionMappingEntry const *> returnEntries;
    for (std::unique_ptr<EnumeratedActionMapEntry> &entry : entries_) {
        if (entry != nullptr) {
            returnEntries.push_back(entry.get());
        }
    }
    return returnEntries;
}

long EnumeratedActionMap::getNChildren() const {
    return nChildren_;
}

bool EnumeratedActionMap::hasRolloutActions() const {
    return actionsToTry_.size() > 0;
}

std::vector<std::unique_ptr<Action>> EnumeratedActionMap::getRolloutActions() const {
    std::vector<std::unique_ptr<Action>> actions;
    for (long code : actionsToTry_) {
        actions.push_back(std::move(allActions_[code]->copy()));
    }
    return actions;
}

void EnumeratedActionMap::updateBestValue() {
    bestAction_ = nullptr;
    bestMeanQValue_ = -std::numeric_limits<double>::infinity();
    if (getNChildren() == 0) {
        debug::show_message("WARNING: No children - could not update Q-value!");
        return;
    }
    for (std::unique_ptr<EnumeratedActionMapEntry> const &entry : entries_) {
        if (entry == nullptr) {
            continue;
        }
        double meanQValue = entry->getActionNode()->getMeanQValue();
        if (bestMeanQValue_ < meanQValue) {
            bestMeanQValue_ = meanQValue;
            bestAction_ = entry->getAction();
        }
    }
}

std::unique_ptr<Action> EnumeratedActionMap::getBestAction() const {
    return bestAction_->copy();
}

double EnumeratedActionMap::getBestMeanQValue() const {
    return bestMeanQValue_;
}

/* ------------------- EnumeratedActionMapEntry ------------------- */
EnumeratedActionMapEntry::EnumeratedActionMapEntry(Action const &action,
        std::unique_ptr<ActionNode> childNode) :
    action_(action.copy()),
    childNode_(std::move(childNode)) {
}

std::unique_ptr<Action> EnumeratedActionMapEntry::getAction() const {
    return action_->copy();
}

ActionNode *EnumeratedActionMapEntry::getActionNode() const {
    return childNode_.get();
}

/* ------------------- EnumeratedActionTextSerializer ------------------- */
void EnumeratedActionTextSerializer::saveActionPool(
        ActionPool const &/*actionPool*/, std::ostream &/*os*/) {
    // Do nothing - the model can create a new one.
}
std::unique_ptr<ActionPool> EnumeratedActionTextSerializer::loadActionPool(
        std::istream &/*is*/) {
    // Use the model to create a new one.
    return solver_->getModel()->createActionPool();
}

void EnumeratedActionTextSerializer::saveActionMapping(
        ActionMapping const &map, std::ostream &os) {
    EnumeratedActionMap const &enumMap = (
            static_cast<EnumeratedActionMap const &>(map));
    os << enumMap.getNChildren() << " action children" << std::endl;
    os << "Untried (";
    for (std::vector<long>::iterator it = enumMap.actionsToTry_.begin();
            it != enumMap.actionsToTry_.end(); it++) {
        os << *enumMap.allActions_[*it];
        if (std::next(it) != enumMap.actionsToTry_.end()) {
            os << ", ";
        }
    }
    os << ")" << std::endl;
    for (std::unique_ptr<EnumeratedActionMapEntry> const &entry : enumMap.entries_) {
        if (entry != nullptr) {
            saveAction(entry->getAction().get(), os);
            os << " ";
            save(*entry->getActionNode(), os);
            os << std::endl;
        }
    }
}

std::unique_ptr<ActionMapping>
EnumeratedActionTextSerializer::loadActionMapping(std::istream &is) {
    std::unique_ptr<ActionMapping> map(
            solver_->getActionPool()->createActionMapping());

    EnumeratedActionMap &enumMap = static_cast<EnumeratedActionMap &>(*map);
    enumMap.actionsToTry_.reset();

    std::string line;

    std::getline(is, line);
    std::istringstream(line) >> enumMap.nChildren_;

    std::getline(is, line);
    std::istringstream sstr(line);
    std::string tmpStr;
    std::getline(sstr, tmpStr, '(');
    std::getline(sstr, tmpStr, ')');
    if (tmpStr == "") {
        continue;
    }
    std::istringstream sstr2(tmpStr);
    std::string actionString;
    while (!sstr2.eof()) {
        std::getline(sstr2, actionString, ',');
        std::istringstream sstr3(actionString);
        std::unique_ptr<Action> action = loadAction(sstr3);
        long code = static_cast<EnumeratedPoint const &>(*action).getCode();
        enumMap.actionsToTry_.add(code);
    }

    for (long i = 0; i < enumMap.nChildren_; i++) {
        std::getline(is, line);
        std::istringstream sstr2(line);
        std::unique_ptr<Action> action(loadAction(sstr2));
        std::unique_ptr<ActionNode> actionNode(std::make_unique<ActionNode>());
        load(*actionNode, sstr2);
        long code = static_cast<const EnumeratedPoint &>(*action).getCode();
        enumMap.entries_[code] = std::make_unique<EnumeratedActionMapEntry>(
                *action, std::move(actionNode));
    }
    return std::move(map);
}
} /* namespace solver */



