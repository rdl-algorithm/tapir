#include "enumerated_actions.hpp"

#include <algorithm>
#include <cmath>

#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"
#include "solver/Model.hpp"

#include "solver/geometry/Action.hpp"
#include "solver/geometry/EnumeratedPoint.hpp"

#include "ActionPool.hpp"
#include "ActionMapping.hpp"
#include "ObservationPool.hpp"

namespace solver {
/* ------------------- ModelWithEnumeratedActions ------------------- */
std::unique_ptr<ActionPool>
    ModelWithEnumeratedActions::createActionPool() {
    return std::make_unique<EnumeratedActionPool>(getRandomGenerator(),
            getAllActionsInOrder());
}

/* --------------------- EnumeratedActionPool --------------------- */
EnumeratedActionPool::EnumeratedActionPool(RandomGenerator *randGen,
        std::vector<std::unique_ptr<EnumeratedPoint>> actions) :
    randGen_(randGen),
    allActions_(std::move(actions)) {
}

std::unique_ptr<ActionMapping>
    EnumeratedActionPool::createActionMapping() {
    return std::make_unique<EnumeratedActionMap>(observationPool_,
            allActions_, generateActionOrder());
}

std::vector<long> EnumeratedActionPool::generateActionOrder() {
    std::vector<long> actions(allActions_.size());
    for (long i = 0; i < (long)actions.size(); i++) {
        actions[i] = i;
    }
    std::shuffle(actions.begin(), actions.end(), *randGen_);
    return actions;
}

/* ---------------------- EnumeratedActionMap ---------------------- */
EnumeratedActionMap::EnumeratedActionMap(ObservationPool *observationPool,
        std::vector<std::unique_ptr<EnumeratedPoint>> const &allActions,
        std::vector<long> actionOrder) :
                allActions_(allActions),
                observationPool_(observationPool),
                children_(allActions_.size()),
                nChildren_(0),
                actionOrder_(actionOrder),
                nextActionIterator_(actionOrder_.cbegin()),
                bestAction_(nullptr),
                bestMeanQValue_(-std::numeric_limits<double>::infinity()) {
}

ActionNode* EnumeratedActionMap::getActionNode(Action const &action) const {
    long code = static_cast<EnumeratedPoint const &>(action).getCode();
    return children_[code].get();
}

ActionNode* EnumeratedActionMap::createActionNode(Action const &action) {
    long code = static_cast<EnumeratedPoint const &>(action).getCode();
    children_[code] = std::make_unique<ActionNode>(
            observationPool_->createObservationMapping(), &action);
    nChildren_++;
    return children_[code].get();
}

long EnumeratedActionMap::getNChildren() const{
    return nChildren_;
}

long EnumeratedActionMap::size() const {
    return allActions_.size();
}

bool EnumeratedActionMap::hasActionToTry() const {
    return nextActionIterator_ != actionOrder_.cend();
}

std::unique_ptr<Action> EnumeratedActionMap::getNextActionToTry() {
    return allActions_[*(nextActionIterator_++)]->copy();
}

std::unique_ptr<Action> EnumeratedActionMap::getSearchAction(
        double exploreCoefficient) {
    double maxVal = -std::numeric_limits<double>::infinity();
    Action const *bestAction = nullptr;
    for (std::unique_ptr<ActionNode> const &node : children_) {
        if (node == nullptr) {
            continue;
        }
        double tmpVal = node->getMeanQValue() + exploreCoefficient * std::sqrt(
                std::log(getNChildren()) / node->getNParticles());
        if (maxVal < tmpVal) {
            maxVal = tmpVal;
            bestAction = node->getAction();
        }
    }
    return bestAction->copy();
}

void EnumeratedActionMap::updateBestValue() {
    bestAction_ = nullptr;
    bestMeanQValue_ = -std::numeric_limits<double>::infinity();
    if (getNChildren() == 0) {
        std::cerr << "No children - could not update Q-value!" << std::endl;
        return;
    }
    for (std::unique_ptr<ActionNode> const &node : children_) {
        if (node == nullptr) {
            continue;
        }
        double meanQValue = node->getMeanQValue();
        if (bestMeanQValue_ < meanQValue) {
            bestMeanQValue_ = meanQValue;
            bestAction_ = node->getAction();
        }
    }
}

std::unique_ptr<Action> EnumeratedActionMap::getBestAction() const {
    return bestAction_->copy();
}

double EnumeratedActionMap::getBestMeanQValue() const {
    return bestMeanQValue_;
}

std::vector<ActionNode *> EnumeratedActionMap::getChildren() const {
    std::vector<ActionNode *> children;
    for (std::unique_ptr<ActionNode> const &node : children_) {
        children.push_back(node.get());
    }
    return children;
}

/* ------------------- EnumeratedActionTextSerializer ------------------- */
void EnumeratedActionTextSerializer::saveActionPool(
        ActionPool const &/*actionPool*/, std::ostream &/*os*/) {
    // Do nothing - the model can create a new one.
}
std::unique_ptr<ActionPool> EnumeratedActionTextSerializer::loadActionPool(
        std::istream &/*is*/) {
    // Use the model to create a new one.
    return solver_->model_->createActionPool();
}

void EnumeratedActionTextSerializer::saveActionMapping(
        ActionMapping const &map, std::ostream &os) {
    EnumeratedActionMap const &enumMap = (
            static_cast<EnumeratedActionMap const &>(map));
    std::vector<long>::const_iterator it = enumMap.actionOrder_.cbegin();
    os << "TRIED   (";
    for (; it != enumMap.nextActionIterator_; it++) {
        os << *enumMap.allActions_[*it];
        if (std::next(it) != enumMap.nextActionIterator_) {
            os << ", ";
        }
    }
    os << ")" << std::endl << "UNTRIED (";
    for (; it != enumMap.actionOrder_.cend(); it++) {
        os << *enumMap.allActions_[*it];
        if (std::next(it) != enumMap.actionOrder_.cend()) {
            os << ", ";
        }
    }
    os << ")" << std::endl;
    os << enumMap.getNChildren() << std::endl;
    for (std::unique_ptr<ActionNode> const &child : enumMap.children_) {
        if (child != nullptr) {
            save(*child, os);
            os << std::endl;
        }
    }
}

std::unique_ptr<ActionMapping>
EnumeratedActionTextSerializer::loadActionMapping(std::istream &is) {
    std::unique_ptr<EnumeratedActionMap> map(
            static_cast<EnumeratedActionMap *>(
            solver_->actionPool_->createActionMapping().release()));

    std::string line;
    std::vector<long>::iterator codeIterator = map->actionOrder_.begin();
    for (int i = 0; i < 2; i++) {
        std::getline(is, line);
        std::stringstream sstr(line);
        std::string tmpStr;
        if (i == 1) {
            map->nextActionIterator_ = codeIterator;
        }
        std::getline(sstr, tmpStr, '(');
        std::getline(sstr, tmpStr, ')');
        if (tmpStr == "") {
            continue;
        }
        std::stringstream sstr2(tmpStr);
        std::string actionString;
        while (!sstr2.eof()) {
            std::getline(sstr2, actionString, ',');
            std::stringstream sstr3(actionString);
            std::unique_ptr<Action> action = loadAction(sstr3);
            long code = static_cast<EnumeratedPoint const &>(*action).getCode();
            *(codeIterator++) = code;
        }
    }

    std::getline(is, line);
    std::stringstream sstr(line);
    sstr >> map->nChildren_;

    for (long i = 0; i < map->nChildren_; i++) {
        std::getline(is, line);
        std::stringstream sstr2(line);
        std::unique_ptr<ActionNode> actionNode(std::make_unique<ActionNode>());
        load(*actionNode, sstr2);
        long code = static_cast<const EnumeratedPoint *>(
                actionNode->getAction())->getCode();
        map->children_[code] = std::move(actionNode);
    }
    return std::move(map);
}
} /* namespace solver */



