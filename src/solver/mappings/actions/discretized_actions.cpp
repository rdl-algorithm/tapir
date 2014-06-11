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

#include "ActionMapping.hpp"
#include "ActionMappingEntry.hpp"
#include "ActionPool.hpp"

namespace solver {
/* ---------------------- DiscretizedActionPool ---------------------- */
DiscretizedActionPool::DiscretizedActionPool(Model *model) :
        model_(model) {
}
std::unique_ptr<ActionMapping> DiscretizedActionPool::createActionMapping(BeliefNode *node) {
    return std::make_unique<DiscretizedActionMap>(node, this,
            createBinSequence(node->getHistoricalData()));
}

/* ---------------------- DiscretizedActionMap ---------------------- */
DiscretizedActionMap::DiscretizedActionMap(BeliefNode *owner, DiscretizedActionPool *pool,
        std::vector<long> binSequence) :
        ActionMapping(owner),
                model_(pool->model_),
                pool_(pool),
                numberOfBins_(pool_->getNumberOfBins()),
                entries_(numberOfBins_),
                nChildren_(0),
                numberOfVisitedEntries_(0),
                binSequence_(std::move(binSequence)),
                binSequenceIndex_(0),
                totalVisitCount_(0) {
    for (int i = 0; i < numberOfBins_; i++) {
        DiscretizedActionMapEntry &entry = entries_[i];
        entry.binNumber_ = i;
        entry.map_ = this;
    }
}

// Default destructor.
DiscretizedActionMap::~DiscretizedActionMap() {
}

ActionNode* DiscretizedActionMap::getActionNode(Action const &action) const {
    long code = static_cast<DiscretizedPoint const &>(action).getBinNumber();
    return entries_[code].getActionNode();
}
ActionNode* DiscretizedActionMap::createActionNode(Action const &action) {
    long code = static_cast<DiscretizedPoint const &>(action).getBinNumber();
//    if (code != *binIterator_) {
//        debug::show_message("WARNING: Trying actions out of order!?");
//    }

    DiscretizedActionMapEntry &entry = entries_[code];

    std::unique_ptr<ActionNode> actionNode = std::make_unique<ActionNode>(&entry);
    ActionNode *node = actionNode.get();
    entry.childNode_ = std::move(actionNode);

    nChildren_++;
    return node;
}
long DiscretizedActionMap::getNChildren() const {
    return nChildren_;
}


long DiscretizedActionMap::getNumberOfVisitedEntries() const {
    return numberOfVisitedEntries_;
}
std::vector<ActionMappingEntry const *> DiscretizedActionMap::getVisitedEntries() const {
    std::vector<ActionMappingEntry const *> returnEntries;
    for (DiscretizedActionMapEntry const &entry : entries_) {
        if (entry.visitCount_ > 0) {
            returnEntries.push_back(&entry);
        }
    }
    return returnEntries;
}
ActionMappingEntry *DiscretizedActionMap::getEntry(Action const &action) {
    long code = static_cast<DiscretizedPoint const &>(action).getBinNumber();
    return &entries_[code];
}
ActionMappingEntry const *DiscretizedActionMap::getEntry(Action const &action) const {
    long code = static_cast<DiscretizedPoint const &>(action).getBinNumber();
    return &entries_[code];
}

std::unique_ptr<Action> DiscretizedActionMap::getNextActionToTry() {
    while (binSequenceIndex_ < binSequence_.size()) {
        long binNumber = binSequence_[binSequenceIndex_];
        // Only return unvisited actions.
        if (entries_[binNumber].visitCount_ <= 0) {
            return pool_->sampleAnAction(binNumber);
        }
        binSequenceIndex_++;
    }
    // No more actions to try!
    return nullptr;
}

long DiscretizedActionMap::getTotalVisitCount() const {
    return totalVisitCount_;
}

/* ------------------- DiscretizedActionMapEntry ------------------- */
/** Returns the mapping this entry belongs to. */
ActionMapping *DiscretizedActionMapEntry::getMapping() const {
    return map_;
}
std::unique_ptr<Action> DiscretizedActionMapEntry::getAction() const {
    return map_->pool_->sampleAnAction(binNumber_);
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

bool DiscretizedActionMapEntry::updateValue(long deltaNVisits, double deltaTotalQ) {
    if (deltaNVisits == 0 && deltaTotalQ == 0) {
        return false;
    }

    if (!std::isfinite(deltaTotalQ)) {
        debug::show_message("ERROR: Non-finite delta value!");
    }

    // Update the visit counts
    if (visitCount_ == 0 && deltaNVisits > 0) {
        map_->numberOfVisitedEntries_++;
    }
    visitCount_ += deltaNVisits;
    map_->totalVisitCount_ += deltaNVisits;
    if (visitCount_ == 0 && deltaNVisits < 0) {
        map_->numberOfVisitedEntries_--;
        // We've reduced the visit count to 0, so we'll have to try this action again.
        map_->binSequence_.push_back(binNumber_);
    }

    // Update the total Q
    totalQValue_ += deltaTotalQ;

    // Update the mean Q
    double oldMeanQ = meanQValue_;
    if (visitCount_ <= 0) {
        meanQValue_ = -std::numeric_limits<double>::infinity();
    } else {
        meanQValue_ = totalQValue_ / visitCount_;
    }
    return meanQValue_ != oldMeanQ;
}

/* ------------------- DiscretizedActionTextSerializer ------------------- */
void DiscretizedActionTextSerializer::saveActionPool(
        ActionPool const &/*actionPool*/, std::ostream &/*os*/) {
    // Do nothing - the model can create a new one.
}
std::unique_ptr<ActionPool> DiscretizedActionTextSerializer::loadActionPool(
        std::istream &/*is*/) {
    // Use the model to create a new one.
    return getSolver()->getModel()->createActionPool(getSolver());
}

void DiscretizedActionTextSerializer::saveActionMapping(
        ActionMapping const &map, std::ostream &os) {
    DiscretizedActionMap const &discMap = (
            static_cast<DiscretizedActionMap const &>(map));
    os << discMap.getNumberOfVisitedEntries() << " visited actions with ";
    os << discMap.getNChildren() << " children; ";
    os << discMap.getTotalVisitCount() << " visits" << std::endl;

    os << "Untried (";
    for (unsigned int index = discMap.binSequenceIndex_;
            index < discMap.binSequence_.size(); index++) {
        os << discMap.binSequence_[index];
        if (index + 1 < discMap.binSequence_.size()) {
            os << ", ";
        } else {
            break;
        }
    }
    os << ")" << std::endl;
    std::multimap<std::pair<double, double>, DiscretizedActionMapEntry const *> entriesByValue;
    long visitedCount = 0;
    for (DiscretizedActionMapEntry const &entry : discMap.entries_) {
        if (entry.visitCount_ > 0) {
            visitedCount++;
            entriesByValue.emplace(
                    std::make_pair(entry.meanQValue_, entry.binNumber_), &entry);
        }
    }
    if (visitedCount != discMap.getNumberOfVisitedEntries()) {
        debug::show_message("ERROR: incorrect number of visited entries!");
    }

    for (auto it = entriesByValue.rbegin(); it != entriesByValue.rend(); it++) {
        DiscretizedActionMapEntry const &entry = *it->second;
        os << "Action " << entry.getBinNumber() << " (";
        saveAction(entry.getAction().get(), os);
        os << "): " << entry.getMeanQValue() << " from ";
        os << entry.getVisitCount() << " visits; total: ";
        os << entry.getTotalQValue();
        ActionNode *node = entry.getActionNode();
        if (node == nullptr) {
            os << " NO CHILD" << std::endl;
        } else {
            os << std::endl;
            save(*entry.getActionNode(), os);
        }
    }
}

std::unique_ptr<ActionMapping>
DiscretizedActionTextSerializer::loadActionMapping(BeliefNode *owner, std::istream &is) {
    std::unique_ptr<DiscretizedActionMap> discMap = std::make_unique<DiscretizedActionMap>(
            owner,
            static_cast<DiscretizedActionPool *>(getSolver()->getActionPool()),
            std::vector<long> { });

    std::string line;
    std::getline(is, line);
    std::string tmpStr;
    std::istringstream sstr4(line);

    sstr4 >> discMap->numberOfVisitedEntries_ >> tmpStr >> tmpStr >> tmpStr;
    sstr4 >> discMap->nChildren_ >> tmpStr;
    sstr4 >> discMap->totalVisitCount_;

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
            discMap->binSequence_.push_back(code);
        }
    }
    discMap->binSequenceIndex_ = 0;

    for (long i = 0; i < discMap->numberOfVisitedEntries_ ; i++) {
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
        sstr2 >> totalQValue;

        bool hasChild = true;
        std::string tmpStr1, tmpStr2;
        sstr2 >> tmpStr1 >> tmpStr2;
        if (tmpStr1 == "NO" && tmpStr2 == "CHILD") {
            hasChild = false;
        }

        // Create an entry to hold the action node.
        DiscretizedActionMapEntry &entry = discMap->entries_[binNumber];
        entry.binNumber_ = binNumber;
        entry.map_ = discMap.get();
        entry.meanQValue_ = meanQValue;
        entry.visitCount_ = visitCount;
        entry.totalQValue_ = totalQValue;

        // Read in the action node itself.
        if (hasChild) {
            entry.childNode_ = std::make_unique<ActionNode>(&entry);
            ActionNode *node = entry.childNode_.get();
            load(*node, is);
        }
    }
    return std::move(discMap);
}

} /* namespace solver */



