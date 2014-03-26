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
                bestBinNumber_(-1),
                bestMeanQValue_(-std::numeric_limits<double>::infinity()),
                totalVisitCount_(0) {
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
ActionMappingEntry const *DiscretizedActionMap::getEntry(Action const &action) const {
    long code = static_cast<DiscretizedPoint const &>(action).getBinNumber();
    return entries_[code].get();
}


long DiscretizedActionMap::getTotalVisitCount() const {
    return totalVisitCount_;
}
std::unique_ptr<Action> DiscretizedActionMap::getBestAction() const {
    if (bestBinNumber_ == -1) {
        return nullptr;
    }
    return model_->sampleAnAction(bestBinNumber_);
}
double DiscretizedActionMap::getBestMeanQValue() const {
    return bestMeanQValue_;
}


bool DiscretizedActionMap::hasUnvisitedActions() const {
    return binsToTry_.size() > 0;
}
std::vector<std::unique_ptr<Action>> DiscretizedActionMap::getUnvisitedActions() const {
    std::vector<std::unique_ptr<Action>> actions;
    for (long binNumber : binsToTry_) {
        actions.push_back(std::move(model_->sampleAnAction(binNumber)));
    }
    return actions;
}
std::unique_ptr<Action> DiscretizedActionMap::getRandomUnvisitedAction() const {
    if (binsToTry_.size() == 0) {
        debug::show_message("Attempted rollout but no actions!?");
        return nullptr;
    }
    int randomIndex = std::uniform_int_distribution<int>(0, binsToTry_.size()-1)(*model_->getRandomGenerator());
    return model_->sampleAnAction(binsToTry_.get(randomIndex));
}

void DiscretizedActionMap::deleteUnvisitedAction(long binNumber) {
    binsToTry_.remove(binNumber);
}
void DiscretizedActionMap::addUnvisitedAction(long binNumber) {
    binsToTry_.add(binNumber);
}

long DiscretizedActionMap::getVisitCount(Action const &action) const {
    long code = static_cast<DiscretizedPoint const &>(action).getBinNumber();
    return entries_[code]->getVisitCount();
}
double DiscretizedActionMap::getTotalQValue(Action const &action) const {
    long code = static_cast<DiscretizedPoint const &>(action).getBinNumber();
    return entries_[code]->getTotalQValue();
}
double DiscretizedActionMap::getMeanQValue(Action const &action) const {
    long code = static_cast<DiscretizedPoint const &>(action).getBinNumber();
    return entries_[code]->getMeanQValue();
}

void DiscretizedActionMap::updateVisitCount(Action const &action, long deltaNVisits) {
    long code = static_cast<DiscretizedPoint const &>(action).getBinNumber();
    DiscretizedActionMapEntry &entry = *entries_[code];
    if (entry.visitCount_ == 0 && deltaNVisits > 0) {
        deleteUnvisitedAction(code);
    }
    entry.visitCount_ += deltaNVisits;
    totalVisitCount_ += deltaNVisits;
    if (entry.visitCount_ == 0 && deltaNVisits < 0) {
        addUnvisitedAction(code);
    }

    if (entry.visitCount_ <= 0) {
        entry.meanQValue_ = -std::numeric_limits<double>::infinity();
    } else {
        entry.meanQValue_ = entry.totalQValue_ / entry.visitCount_;
    }
}
void DiscretizedActionMap::updateTotalQValue(Action const &action, double deltaQ) {
    long code = static_cast<DiscretizedPoint const &>(action).getBinNumber();
    DiscretizedActionMapEntry &entry = *entries_[code];
    entry.totalQValue_ += deltaQ;
    if (entry.visitCount_ <= 0) {
        entry.meanQValue_ = -std::numeric_limits<double>::infinity();
    } else {
        entry.meanQValue_ = entry.totalQValue_ / entry.visitCount_;
    }
}
void DiscretizedActionMap::updateQValue() {
    bestBinNumber_ = -1;
    bestMeanQValue_ = -std::numeric_limits<double>::infinity();
    for (std::unique_ptr<DiscretizedActionMapEntry> const &entry : entries_) {
        if (entry == nullptr) {
            continue;
        }
        if (entry->visitCount_ <= 0) {
            continue;
        }
        double meanQValue = entry->meanQValue_;
        if (bestMeanQValue_ < meanQValue) {
            bestMeanQValue_ = meanQValue;
            bestBinNumber_ = entry->getBinNumber();
        }
    }
}

/* ------------------- DiscretizedActionMapEntry ------------------- */
DiscretizedActionMapEntry::DiscretizedActionMapEntry(long binNumber,
        DiscretizedActionMap *map,
        std::unique_ptr<ActionNode> childNode) :
                binNumber_(binNumber),
                map_(map),
                childNode_(std::move(childNode)),
                visitCount_(0),
                totalQValue_(0),
                meanQValue_(-std::numeric_limits<double>::infinity()) {
}

std::unique_ptr<Action> DiscretizedActionMapEntry::getAction() const {
    return map_->model_->sampleAnAction(binNumber_);
}
ActionNode *DiscretizedActionMapEntry::getActionNode() const {
    return childNode_.get();
}
long DiscretizedActionMapEntry::getVisitCount() const {
    return visitCount_;
}
double DiscretizedActionMapEntry::getTotalQValue() const {
    return totalQValue_;
}
double DiscretizedActionMapEntry::getMeanQValue() const {
    return meanQValue_;
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
    os << discMap.getNChildren() << " action children; ";
    os << discMap.getTotalVisitCount() << " visits" << std::endl;
    os << "Untried (";
    for (std::vector<long>::const_iterator it = discMap.binsToTry_.begin();
            it != discMap.binsToTry_.end(); it++) {
        os << *it;
        if (std::next(it) != discMap.binsToTry_.end()) {
            os << ", ";
        }
    }
    os << ")" << std::endl;
    std::multimap<std::pair<double, double>, DiscretizedActionMapEntry const *> entriesByValue;
    for (std::unique_ptr<DiscretizedActionMapEntry> const &entry : discMap.entries_) {
        if (entry != nullptr) {
            entriesByValue.emplace(std::make_pair(entry->meanQValue_, entry->binNumber_), entry.get());
        }
    }

    for (auto it = entriesByValue.rbegin(); it != entriesByValue.rend(); it++) {
        DiscretizedActionMapEntry const &entry = *it->second;
        os << "Action " << entry.getBinNumber() << " (";
        saveAction(entry.getAction().get(), os);
        os << "): " << entry.getMeanQValue() << " from ";
        os << entry.getVisitCount() << " visits; total: ";
        os << entry.getTotalQValue() << std::endl;
        save(*entry.getActionNode(), os);
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
    std::string tmpStr;
    std::istringstream sstr4(line);
    sstr4 >> discMap.nChildren_ >> tmpStr >> tmpStr;
    sstr4 >> discMap.totalVisitCount_;

    std::getline(is, line);
    std::istringstream sstr(line);
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
        // The first line contains info from the mapping.
        std::getline(is, line);
        std::istringstream sstr2(line);
        long binNumber;
        sstr2 >> tmpStr >> binNumber;
        std::getline(sstr2, tmpStr, '(');
        std::getline(sstr2, tmpStr, ')');
        std::getline(sstr2, tmpStr, ':');
        double meanQValue, totalQValue;
        long visitCount;
        sstr2 >> meanQValue >> tmpStr;
        sstr2 >> visitCount >> tmpStr >> tmpStr;
        sstr2 >> totalQValue >> tmpStr;

        // Now we read the action node.
        std::unique_ptr<ActionNode> actionNode = std::make_unique<ActionNode>();
        load(*actionNode, is);
        std::unique_ptr<DiscretizedActionMapEntry> entry = std::make_unique<DiscretizedActionMapEntry>(
                binNumber, &discMap, std::move(actionNode));
        entry->meanQValue_ = meanQValue;
        entry->visitCount_ = visitCount;
        entry->totalQValue_ = totalQValue;
        discMap.entries_[binNumber] = std::move(entry);
    }
    return std::move(map);
}
} /* namespace solver */



