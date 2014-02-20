#include "enumerated_actions.hpp"

#include <cmath>

#include <iostream>
#include <memory>
#include <vector>

#include "ActionPool.hpp"
#include "ActionMapping.hpp"
#include "BeliefNode.hpp"
#include "EnumeratedPoint.hpp"
#include "Model.hpp"
#include "ObservationPool.hpp"

namespace solver {
/* ------------------- ModelWithEnumeratedActions ------------------- */
std::unique_ptr<ActionPool>
    ModelWithEnumeratedActions::createActionPool() {
    return std::make_unique<ActionPool>(getAllActions());
}

/* --------------------- EnumeratedActionPool --------------------- */
EnumeratedActionPool::EnumeratedActionPool(
        std::vector<std::unique_ptr<EnumeratedPoint>> actions) :
    actions_(std::move(actions)) {
}

EnumeratedActionPool::~EnumeratedActionPool() {
}

std::unique_ptr<ActionMapping>
    EnumeratedActionPool::createActionMapping() {
    return std::make_unique<EnumeratedActionMap>(observationPool_, actions_);
}


/* ---------------------- EnumeratedActionMap ---------------------- */
EnumeratedActionMap::EnumeratedActionMap(ObservationPool *observationPool,
        std::vector<std::unique_ptr<EnumeratedPoint>> const &actions,
        std::vector<long> actionOrder) :
                observationPool_(observationPool),
                actions_(actions),
                children_(actions_.size()),
                nChildren_(0),
                actionOrder_(actionOrder),
                nextActionIterator_(actionOrder_.cbegin()) {
}

BeliefNode* EnumeratedActionMap::getBelief(
        Action const &obs) const {
    long code = static_cast<EnumeratedPoint const &>(obs).getCode();
    return children_[code].get();
}

BeliefNode* EnumeratedActionMap::createBelief(
        const Action& obs) {
    long code = static_cast<EnumeratedPoint const &>(obs).getCode();
    children_[code] = std::make_unique<ActionNode>(
            observationPool_->createObservationMapping());
    nChildren_++;
    return children_[code].get();
}

long EnumeratedActionMap::getNChildren() const{
    return nChildren_;
}

long EnumeratedActionMap::size() const {
    return actions_.size();
}


bool EnumeratedActionMap::hasActionToTry() const {
    return nextActionIterator_ != actionOrder_.cend();
}

std::unique_ptr<Action> EnumeratedActionMap::getNextActionToTry(
        RandomGenerator *randGen) {
    long code = *nextActionIterator_;
    nextActionIterator_++;
    return actions_[code]->copy();
}

std::unique_ptr<Action> EnumeratedActionMap::getSearchAction(
        double exploreCoefficient) {
    std::vector<ActionNode>::const_iterator iter = children_.cbegin();
    double maxVal = (iter->getMeanQValue() + exploreCoefficient
                      * std::sqrt(std::log(getNChildren())
                             / iter->getNParticles()));
    Action const *bestAction = iter->action_.get();
    iter++;
    for (; iter != children_.cend(); iter++) {
        double tmpVal = (iter->getMeanQValue() + exploreCoefficient
             * std::sqrt(
                     std::log(getNChildren())
                     / iter->getNParticles()));
        if (maxVal < tmpVal) {
            maxVal = tmpVal;
            bestAction = iter->action_.get();
        }
    }
    return bestAction->copy();
}

void EnumeratedActionMap::updateBestValue() {
    if (getNChildren() == 0) {
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

std::unique_ptr<Action> EnumeratedActionMap::getBestAction() const {
    return bestAction_->copy();
}

double EnumeratedActionMap::getBestMeanQValue() const {
    return bestMeanQValue_;
}


/* ------------------- EnumeratedActionTextSerializer ------------------- */
void EnumeratedActionTextSerializer::saveActionPool(
        ActionPool const &actionPool, std::ostream &os) {
    // Do nothing - the model can create a new one.
}
std::unique_ptr<ActionPool> EnumeratedActionTextSerializer::loadActionPool(
        std::istream &is) {
    // Use the model to create a new one.
    return solver_->model_->createActionPool();
}

void EnumeratedActionTextSerializer::saveActionMapping(
        ActionMapping const &map, std::ostream &os) {
    EnumeratedActionMap const &enumMap = (
            static_cast<EnumeratedActionMap const &>(map));
    os << enumMap.getNChildren() << std::endl;
    for (std::unique_ptr<ActionNode> &child : enumMap.children_) {
        if (child != nullptr) {
            save(*child, os);
            os << std::endl;
        }
    }
}

std::unique_ptr<ActionMapping>
EnumeratedActionTextSerializer::loadActionMapping(std::istream &is) {
    std::unique_ptr<EnumeratedActionMap> map(
            solver_->actionPool_->createActionMapping());

    std::string line;
    std::getline(is, line);
    std::stringstream sstr(line);
    sstr >> map->nChildren_;

    for (long i = 0; i < map->nChildren_; i++) {
        std::getline(is, line);
        std::stringstream sstr2(line);
        std::unique_ptr<ActionNode> actionNode(std::make_unique<ActionNode>());
        load(*actionNode, sstr2);
        long code = static_cast<EnumeratedPoint *>(
                actionNode->action_.get())->getCode();
        map->children_[code] = std::move(actionNode);
    }
    return std::move(map);
}
} /* namespace solver */



