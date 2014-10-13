/** @file continuous_actions.cpp
 *
 * Contains the implementations of the classes for continuous action mappings.
 */
#include "solver/mappings/actions/continuous_actions.hpp"

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

#include "solver/mappings/actions/ActionMapping.hpp"
#include "solver/mappings/actions/ActionMappingEntry.hpp"
#include "solver/mappings/actions/ActionPool.hpp"

namespace solver {



/* ---------------------- ContinuousActionPool ---------------------- */
std::unique_ptr<ActionMapping> ContinuousActionPool::createActionMapping(BeliefNode *node) {
    return std::make_unique<ContinuousActionMap>(node, this);
}

std::unique_ptr<Action> ContinuousActionPool::createAction(const double* constructionDataVector, const BeliefNode* belief) const {
	return createAction(*createActionConstructionData(constructionDataVector, belief));
}

std::shared_ptr<const std::vector<ContinuousActionConstructionDataBase>> ContinuousActionPool::createFixedActions(const BeliefNode* /*belief*/) const {
	return nullptr;
}

bool ContinuousActionPool::randomiseFixedActions(const BeliefNode* /*belief*/) const {
	return true;
}


/* ---------------------- ChooserDataBase ---------------------- */
namespace ChooserDataBase_detail {

void ChooserDataBaseBase::registerDerivedType(const std::string& name, const LoadFromStreamFunction& loader) {
	getDerivedLoadersSingleton()[name] = loader;
}

std::unordered_map<std::string, ChooserDataBaseBase::LoadFromStreamFunction>& ChooserDataBaseBase::getDerivedLoadersSingleton() {
	static std::unordered_map<std::string, ChooserDataBaseBase::LoadFromStreamFunction> singleton;
	return singleton;
}

} // namespace ChooserDataBase_detail


/* ---------------------- ContinuousActionMap ---------------------- */
ContinuousActionMap::ContinuousActionMap(BeliefNode *owner, ContinuousActionPool *thePool):
        ActionMapping(owner),
        pool(thePool),
        entries(pool->createActionContainer(owner)) {
}


const ContinuousActionPool* ContinuousActionMap::getActionPool() const {
	return pool;
}

ActionNode* ContinuousActionMap::getActionNode(Action const &baseAction) const {
	const ThisAction& action = static_cast<const ThisAction&>(baseAction);
	return entries->at(action.getConstructionData())->getActionNode();
}

ActionNode* ContinuousActionMap::createActionNode(Action const &baseAction) {

	const ThisAction& action = static_cast<const ThisAction&>(baseAction);

	ThisActionMapEntry* entry = entries->at(action.getConstructionData()).get();

	std::unique_ptr<ActionNode> actionNode = std::make_unique<ActionNode>(entry);
	ActionNode *node = actionNode.get();
	entry->setChild(std::move(actionNode));

	nChildren++;

	return node;
}

long ContinuousActionMap::getNChildren() const {
	return nChildren;
}

void ContinuousActionMap::deleteChild(ActionMappingEntry const *entry) {
	ThisActionMapEntry &discEntry = const_cast<ThisActionMapEntry &>(static_cast<ThisActionMapEntry const &>(*entry));
	discEntry.deleteChild();
}

std::vector<ActionMappingEntry const *> ContinuousActionMap::getChildEntries() const  {
	return entries->getEntriesWithChildren();
}


long ContinuousActionMap::getNumberOfVisitedEntries() const {
	return numberOfVisitedEntries;
}

std::vector<ActionMappingEntry const *> ContinuousActionMap::getVisitedEntries() const {
	return entries->getEntriesWithNonzeroVisitCount();
}

ActionMappingEntry *ContinuousActionMap::getEntry(Action const &baseAction) {
	const ThisAction& action = static_cast<const ThisAction&>(baseAction);
	return entries->at(action.getConstructionData()).get();
}

ActionMappingEntry const *ContinuousActionMap::getEntry(Action const &baseAction) const {
	const ThisAction& action = static_cast<const ThisAction&>(baseAction);
	return entries->at(action.getConstructionData()).get();
}

long ContinuousActionMap::getTotalVisitCount() const {
	return totalVisitCount;
}

std::unique_ptr<Action> ContinuousActionMap::getNextActionToTry() {
	class NoNextActionToTry: public std::exception {
		virtual const char* what() const noexcept {
			return "ContinuousActionMap::getNextActionToTry() isn't implemented. It doesn't make much sense for continuous actions. Please use a stepper/chooser that doesn't rely on this feature.";
		}
	} e;
	throw e;
	return nullptr;
}


/* ------------------- ContinuousActionMapEntry ------------------- */

ContinuousActionMapEntry::ContinuousActionMapEntry(ThisActionMap* theMap, std::unique_ptr<ThisActionConstructionData>&& theConstructionData, bool theIsLegal):
		map(theMap),
		constructionData(std::move(theConstructionData)),
		isLegal_(theIsLegal) {}


ActionMapping *ContinuousActionMapEntry::getMapping() const {
    return map;
}
std::unique_ptr<Action> ContinuousActionMapEntry::getAction() const {
	return map->getActionPool()->createAction(*constructionData);
}
ActionNode* ContinuousActionMapEntry::getActionNode() const {
    return childNode.get();
}
long ContinuousActionMapEntry::getVisitCount() const {
    return visitCount_;
}
double ContinuousActionMapEntry::getTotalQValue() const {
    return totalQValue_;
}
double ContinuousActionMapEntry::getMeanQValue() const {
    return meanQValue_;
}
bool ContinuousActionMapEntry::isLegal() const {
    return isLegal_;
}


bool ContinuousActionMapEntry::update(long deltaNVisits, double deltaTotalQ) {
	if (deltaNVisits == 0 && deltaTotalQ == 0) {
		return false;
	}

	if (!std::isfinite(deltaTotalQ)) {
		debug::show_message("ERROR: Non-finite delta value!");
	}

	if (deltaNVisits > 0 && !isLegal_) {
		debug::show_message("ERROR: Visiting an illegal action!");
	}

	// Update the visit counts
	if (visitCount_ == 0 && deltaNVisits > 0) {
		map->numberOfVisitedEntries++;
	}
	visitCount_ += deltaNVisits;
	map->totalVisitCount += deltaNVisits;
	if (visitCount_ == 0 && deltaNVisits < 0) {
		map->numberOfVisitedEntries--;
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


void ContinuousActionMapEntry::setLegal(bool legal) {
	isLegal_ = legal;
}

void ContinuousActionMapEntry::setChild(std::unique_ptr<ActionNode>&& child) {
	childNode = std::move(child);
}

void ContinuousActionMapEntry::deleteChild() {
	childNode.reset();
}

const ActionNode* ContinuousActionMapEntry::getChild() const {
	return childNode.get();
}

/* ------------------- ContinuousActionTextSerializer ------------------- */
void ContinuousActionTextSerializer::saveActionPool( ActionPool const &/*actionPool*/, std::ostream &/*os*/) {
    // Do nothing - the model can create a new one!
}
std::unique_ptr<ActionPool> ContinuousActionTextSerializer::loadActionPool(std::istream &/*is*/) {
    // Use the model to create a new one.
    return getSolver()->getModel()->createActionPool(getSolver());
}

void ContinuousActionTextSerializer::saveActionMapping(ActionMapping const &map, std::ostream &os) {
    DiscretizedActionMap const &discMap = static_cast<DiscretizedActionMap const &>(map);
    os << discMap.getNumberOfVisitedEntries() << " visited actions with ";
    os << discMap.getNChildren() << " children; ";
    os << discMap.getTotalVisitCount() << " visits" << std::endl;

    os << "Untried (";
    for (auto it = discMap.binSequence_.begin(); it != discMap.binSequence_.end(); it++) {
        os << *it;
        if (std::next(it) != discMap.binSequence_.end()) {
            os << ", ";
        }
    }
    os << ")" << std::endl;
    std::multimap<std::pair<double, double>, DiscretizedActionMapEntry const *> entriesByValue;
    long visitedCount = 0;
    for (int i = 0; i < discMap.numberOfBins_; i++) {
        DiscretizedActionMapEntry const &entry = discMap.entries_[i];
        if (entry.visitCount_ > 0) {
            visitedCount++;
        }
        // An entry is saved if it has a node, or a nonzero visit count.
        if (entry.visitCount_ > 0 || entry.childNode_ != nullptr) {
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
        if (!entry.isLegal()) {
            os << " ILLEGAL";
        }
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
ContinuousActionTextSerializer::loadActionMapping(BeliefNode *owner, std::istream &is) {
    std::unique_ptr<DiscretizedActionMap> discMap = std::make_unique<DiscretizedActionMap>(
            owner,
            static_cast<DiscretizedActionPool *>(getSolver()->getActionPool()),
            std::vector<long> { });
    loadActionMapping(*discMap, is);
    return std::move(discMap);
}

void ContinuousActionTextSerializer::loadActionMapping(DiscretizedActionMap &discMap,
        std::istream &is) {

    std::string line;
    std::getline(is, line);
    std::string tmpStr;
    std::istringstream sstr4(line);

    sstr4 >> discMap.numberOfVisitedEntries_ >> tmpStr >> tmpStr >> tmpStr;
    sstr4 >> discMap.nChildren_ >> tmpStr;
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
            discMap.binSequence_.add(code);
        }
    }

    for (long i = 0; i < discMap.numberOfVisitedEntries_ ; i++) {
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
        std::string legalString;
        sstr2 >> meanQValue >> tmpStr;
        sstr2 >> visitCount >> tmpStr >> tmpStr;
        sstr2 >> totalQValue >> legalString;

        bool hasChild = true;
        std::string tmpStr1, tmpStr2;
        sstr2 >> tmpStr1 >> tmpStr2;
        if (tmpStr1 == "NO" && tmpStr2 == "CHILD") {
            hasChild = false;
        }

        // Create an entry to hold the action node.
        DiscretizedActionMapEntry &entry = discMap.entries_[binNumber];
        entry.binNumber_ = binNumber;
        entry.map_ = &discMap;
        entry.meanQValue_ = meanQValue;
        entry.visitCount_ = visitCount;
        entry.totalQValue_ = totalQValue;
        entry.isLegal_ = (legalString != "ILLEGAL");

        // Read in the action node itself.
        if (hasChild) {
            entry.childNode_ = std::make_unique<ActionNode>(&entry);
            ActionNode *node = entry.childNode_.get();
            load(*node, is);
        }
    }

    // Any bins we are supposed to try must be considered legal.
    for (long binNumber : discMap.binSequence_) {
        discMap.entries_[binNumber].isLegal_ = true;
    }
}

} /* namespace solver */



