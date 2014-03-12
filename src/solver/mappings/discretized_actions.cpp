#include "discretized_actions.hpp"

#include <algorithm>
#include <cmath>

#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <vector>

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"
#include "solver/abstract-problem/Model.hpp"

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/DiscretizedPoint.hpp"

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
            model_, numberOfBins_);
}

/* ---------------------- DiscretizedActionMap ---------------------- */
DiscretizedActionMap::DiscretizedActionMap(ObservationPool *observationPool,
        ModelWithDiscretizedActions *model, long numberOfBins) :
                observationPool_(observationPool),
                model_(model),
                numberOfBins_(numberOfBins),
                entries_(numberOfBins_),
                nChildren_(0),
                binsToTry_(),
                bestBinNumber(-1),
                bestMeanQValue_(-std::numeric_limits<double>::infinity()) {
    for (long i = 0; i < numberOfBins_; i++) {
        binsToTry_.add(i);
    }
}

ActionNode* DiscretizedActionMap::getActionNode(Action const &action) const {
    long code = static_cast<DiscretizedPoint const &>(action).getBinNumber();
    if (entries_[code] == nullptr) {
        return nullptr;
    }
    return entries_[code]->getActionNode();
}
ActionNode* DiscretizedActionMap::createActionNode(Action const &action) {
    long code = static_cast<DiscretizedPoint const &>(action).getBinNumber();
    entries_[code] = std::make_unique<DiscretizedActionMapEntry>(
            code, this, std::make_unique<ActionNode>(
                    observationPool_->createObservationMapping()));
    nChildren_++;
    return entries_[code]->getActionNode();
}

long DiscretizedActionMap::getNChildren() const {
    return nChildren_;
}
std::vector<ActionMappingEntry const *> DiscretizedActionMap::getChildEntries() const {
    std::vector<ActionMappingEntry const *> returnEntries;
    for (std::unique_ptr<DiscretizedActionMapEntry> const &entry : entries_) {
        if (entry != nullptr) {
            returnEntries.push_back(entry.get());
        }
    }
    return returnEntries;
}

bool DiscretizedActionMap::hasRolloutActions() const {
    return binsToTry_.size() > 0;
}

std::vector<std::unique_ptr<Action>> DiscretizedActionMap::getRolloutActions() const {
    std::vector<std::unique_ptr<Action>> actions;
    for (long binNumber : binsToTry_) {
        actions.push_back(std::move(model_->sampleAnAction(binNumber)));
    }
    return actions;
}

std::unique_ptr<Action> DiscretizedActionMap::getRandomRolloutAction() const {
    if (binsToTry_.size() == 0) {
        debug::show_message("Attempted rollout but no actions!?");
        return nullptr;
    }
    int randomIndex = std::uniform_int_distribution<int>(0, binsToTry_.size()-1)(*model_->getRandomGenerator());
    return model_->sampleAnAction(binsToTry_.get(randomIndex));
}

void DiscretizedActionMap::update() {
    bestBinNumber = -1;
    bestMeanQValue_ = -std::numeric_limits<double>::infinity();
    if (getNChildren() == 0) {
        debug::show_message("WARNING: No children - could not update Q-value!");
        return;
    }
    for (std::unique_ptr<DiscretizedActionMapEntry> const &entry : entries_) {
        if (entry == nullptr) {
            continue;
        }
        if (entry->getActionNode()->getNParticles() == 0) {
            binsToTry_.add(entry->getBinNumber());
            continue;
        }
        binsToTry_.remove(entry->getBinNumber());
        double meanQValue = entry->getActionNode()->getQValue();
        if (bestMeanQValue_ < meanQValue) {
            bestMeanQValue_ = meanQValue;
            bestBinNumber = entry->getBinNumber();
        }
    }
}

std::unique_ptr<Action> DiscretizedActionMap::getBestAction() const {
    if (bestBinNumber == -1) {
        return nullptr;
    }
    return model_->sampleAnAction(bestBinNumber);
}

double DiscretizedActionMap::getBestMeanQValue() const {
    return bestMeanQValue_;
}

/* ------------------- DiscretizedActionMapEntry ------------------- */
DiscretizedActionMapEntry::DiscretizedActionMapEntry(long binNumber,
        DiscretizedActionMap *map,
        std::unique_ptr<ActionNode> childNode) :
                binNumber_(binNumber),
                map_(map),
                childNode_(std::move(childNode)) {
}

std::unique_ptr<Action> DiscretizedActionMapEntry::getAction() const {
    return map_->model_->sampleAnAction(binNumber_);
}

ActionNode *DiscretizedActionMapEntry::getActionNode() const {
    return childNode_.get();
}

long DiscretizedActionMapEntry::getBinNumber() const {
    return binNumber_;
}

/* ------------------- DiscretizedActionTextSerializer ------------------- */
void DiscretizedActionTextSerializer::saveActionPool(
        ActionPool const &/*actionPool*/, std::ostream &/*os*/) {
    // Do nothing - the model can create a new one.
}
std::unique_ptr<ActionPool> DiscretizedActionTextSerializer::loadActionPool(
        std::istream &/*is*/) {
    // Use the model to create a new one.
    return solver_->getModel()->createActionPool();
}

void DiscretizedActionTextSerializer::saveActionMapping(
        ActionMapping const &map, std::ostream &os) {
    DiscretizedActionMap const &discMap = (
            static_cast<DiscretizedActionMap const &>(map));
    os << discMap.getNChildren() << " action children" << std::endl;
    os << "Untried (";
    for (std::vector<long>::const_iterator it = discMap.binsToTry_.begin();
            it != discMap.binsToTry_.end(); it++) {
        os << *it;
        if (std::next(it) != discMap.binsToTry_.end()) {
            os << ", ";
        }
    }
    os << ")" << std::endl;
    std::multimap<double, DiscretizedActionMapEntry const *> entriesByValue;
    for (std::unique_ptr<DiscretizedActionMapEntry> const &entry : discMap.entries_) {
        if (entry != nullptr) {
            entriesByValue.emplace(entry->getActionNode()->getQValue(), entry.get());
        }
    }

    for (auto it = entriesByValue.rbegin(); it != entriesByValue.rend(); it++) {
        os << "Action " << it->second->getBinNumber() << " (";
        saveAction(it->second->getAction().get(), os);
        os << "): ";
        save(*it->second->getActionNode(), os);
        os << std::endl;
    }
}

std::unique_ptr<ActionMapping>
DiscretizedActionTextSerializer::loadActionMapping(std::istream &is) {
    std::unique_ptr<ActionMapping> map(
            solver_->getActionPool()->createActionMapping());
    DiscretizedActionMap &discMap = static_cast<DiscretizedActionMap &>(*map);
    discMap.binsToTry_.clear();

    std::string line;

    std::getline(is, line);
    std::istringstream(line) >> discMap.nChildren_;

    std::getline(is, line);
    std::istringstream sstr(line);
    std::string tmpStr;
    std::getline(sstr, tmpStr, '(');
    std::getline(sstr, tmpStr, ')');
    if (tmpStr != "") {
        std::istringstream sstr2(tmpStr);
        std::string actionString;
        while (!sstr2.eof()) {
            std::getline(sstr2, actionString, ',');
            long code;
            std::istringstream(actionString) >> code;
            discMap.binsToTry_.add(code);
        }
    }

    for (long i = 0; i < discMap.nChildren_; i++) {
        std::getline(is, line);
        std::istringstream sstr2(line);
        long binNumber;
        sstr2 >> tmpStr >> binNumber >> tmpStr;
        std::unique_ptr<ActionNode> actionNode(std::make_unique<ActionNode>());
        load(*actionNode, sstr2);
        discMap.entries_[binNumber] = std::make_unique<DiscretizedActionMapEntry>(
                binNumber, &discMap, std::move(actionNode));
    }
    return std::move(map);
}
} /* namespace solver */



