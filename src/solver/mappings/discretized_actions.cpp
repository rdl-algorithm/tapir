#include "discretized_actions.hpp"

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
/* ------------------- ModelWithDiscretizedActions ------------------- */
std::unique_ptr<ActionPool>
    ModelWithDiscretizedActions::createActionPool() {
    return std::make_unique<DiscretizedActionPool>(this,
            getNumberOfBins());
}

/* --------------------- DiscretizedActionPool --------------------- */
DiscretizedActionPool::DiscretizedActionPool(
        ModelWithDiscretizedActions *model, long numberOfBins) :
                model_(model),
                numberOfBins_(numberOfBins) {
}

std::unique_ptr<ActionMapping>
    DiscretizedActionPool::createActionMapping() {
    return std::make_unique<DiscretizedActionMap>(observationPool_,
            model_, numberOfBins_, generateActionOrder());
}

std::vector<long> DiscretizedActionPool::generateActionOrder() {
    std::vector<long> actions(numberOfBins_);
    for (long i = 0; i < (long)actions.size(); i++) {
        actions[i] = i;
    }
    std::shuffle(actions.begin(), actions.end(),
            *model_->getRandomGenerator());
    return actions;
}

/* ---------------------- DiscretizedActionMap ---------------------- */
DiscretizedActionMap::DiscretizedActionMap(ObservationPool *observationPool,
        ModelWithDiscretizedActions *model,
        long numberOfBins, std::vector<long> actionOrder) :
                observationPool_(observationPool),
                model_(model),
                numberOfBins_(numberOfBins),
                children_(numberOfBins_),
                nChildren_(0),
                actionOrder_(actionOrder),
                nextActionIterator_(actionOrder_.cbegin()),
                bestActionCode_(-1),
                bestMeanQValue_(-std::numeric_limits<double>::infinity()) {
}

ActionNode* DiscretizedActionMap::getActionNode(Action const &action) const {
    long code = static_cast<EnumeratedPoint const &>(action).getCode();
    return children_[code].get();
}

ActionNode* DiscretizedActionMap::createActionNode(Action const &action) {
    long code = static_cast<EnumeratedPoint const &>(action).getCode();
    children_[code] = std::make_unique<ActionNode>(
            observationPool_->createObservationMapping(), &action);
    nChildren_++;
    return children_[code].get();
}

long DiscretizedActionMap::getNChildren() const{
    return nChildren_;
}

long DiscretizedActionMap::size() const {
    return numberOfBins_;
}

bool DiscretizedActionMap::hasActionToTry() const {
    return nextActionIterator_ != actionOrder_.cend();
}

std::unique_ptr<Action> DiscretizedActionMap::getNextActionToTry() {
    return model_->sampleAnAction(*nextActionIterator_++);
}

std::unique_ptr<Action> DiscretizedActionMap::getSearchAction(
        double exploreCoefficient) {
    double maxVal = -std::numeric_limits<double>::infinity();
    long bestCode = -1;
    long code = -1;
    for (std::unique_ptr<ActionNode> const &node : children_) {
        code++;
        if (node == nullptr) {
            continue;
        }
        double tmpVal = node->getMeanQValue() + exploreCoefficient * std::sqrt(
                std::log(getNChildren()) / node->getNParticles());
        if (maxVal < tmpVal) {
            maxVal = tmpVal;
            bestCode = code;
        }
    }
    return model_->sampleAnAction(bestCode);
}

void DiscretizedActionMap::updateBestValue() {
    bestActionCode_ = -1;
    bestMeanQValue_ = -std::numeric_limits<double>::infinity();
    if (getNChildren() == 0) {
        debug::show_message("WARNING: No children - could not update Q-value!");
        return;
    }
    long code = -1;
    for (std::unique_ptr<ActionNode> const &node : children_) {
        code++;
        if (node == nullptr) {
            continue;
        }
        double meanQValue = node->getMeanQValue();
        if (bestMeanQValue_ < meanQValue) {
            bestMeanQValue_ = meanQValue;
            bestActionCode_ = code;
        }
    }
}

std::unique_ptr<Action> DiscretizedActionMap::getBestAction() const {
    return model_->sampleAnAction(bestActionCode_);
}

double DiscretizedActionMap::getBestMeanQValue() const {
    return bestMeanQValue_;
}

std::vector<ActionNode *> DiscretizedActionMap::getChildren() const {
    std::vector<ActionNode *> children;
    for (std::unique_ptr<ActionNode> const &node : children_) {
        children.push_back(node.get());
    }
    return children;
}


/* ------------------- DiscretizedActionTextSerializer ------------------- */
void DiscretizedActionTextSerializer::saveActionPool(
        ActionPool const &/*actionPool*/, std::ostream &/*os*/) {
    // Do nothing - the model can create a new one.
}
std::unique_ptr<ActionPool> DiscretizedActionTextSerializer::loadActionPool(
        std::istream &/*is*/) {
    // Use the model to create a new one.
    return solver_->model_->createActionPool();
}

void DiscretizedActionTextSerializer::saveActionMapping(
        ActionMapping const &map, std::ostream &os) {
    DiscretizedActionMap const &enumMap = (
            static_cast<DiscretizedActionMap const &>(map));
    std::vector<long>::const_iterator it = enumMap.actionOrder_.cbegin();
    os << "TRIED   (";
    for (; it != enumMap.nextActionIterator_; it++) {
        os << *it;
        if (std::next(it) != enumMap.nextActionIterator_) {
            os << ", ";
        }
    }
    os << ")" << std::endl << "UNTRIED (";
    for (; it != enumMap.actionOrder_.cend(); it++) {
        os << *it;
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
DiscretizedActionTextSerializer::loadActionMapping(std::istream &is) {
    std::unique_ptr<DiscretizedActionMap> map(
            static_cast<DiscretizedActionMap *>(
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
            long code;
            sstr3 >> code;
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



