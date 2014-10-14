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



/* ---------------------- ContinuousActionContainerBase ---------------------- */

std::vector<ActionMappingEntry const*> ContinuousActionContainerBase::getEntriesWithChildren() const {
	std::vector<ActionMappingEntry const*> result = getEntries();
	auto nextTarget = result.begin();
	for(auto current = result.begin(); current !=  result.end(); ++current) {
		ContinuousActionMapEntry const* entry = static_cast<ContinuousActionMapEntry const*>(*current);
		if (entry->getChild() != nullptr) {
			if (nextTarget != current) {
				*nextTarget = *current;
			}
			++nextTarget;
		}
	}
	result.erase(nextTarget, result.end());
	return std::move(result);
}

std::vector<ActionMappingEntry const*> ContinuousActionContainerBase::getEntriesWithNonzeroVisitCount() const {
	std::vector<ActionMappingEntry const*> result = getEntries();
	auto nextTarget = result.begin();
	for(auto current = result.begin(); current !=  result.end(); ++current) {
		ContinuousActionMapEntry const* entry = static_cast<ContinuousActionMapEntry const*>(*current);
		if (entry->getVisitCount() > 0) {
			if (nextTarget != current) {
				*nextTarget = *current;
			}
			++nextTarget;
		}
	}
	result.erase(nextTarget, result.end());
	return std::move(result);
}




/* ---------------------- ContinuousActionPool ---------------------- */
std::unique_ptr<ActionMapping> ContinuousActionPool::createActionMapping(BeliefNode *node) {
    return std::make_unique<ContinuousActionMap>(node, this);
}

std::unique_ptr<Action> ContinuousActionPool::createAction(const double* constructionDataVector, const BeliefNode* belief) const {
	return createAction(*createActionConstructionData(constructionDataVector, belief));
}

std::vector<std::unique_ptr<ContinuousActionConstructionDataBase>> ContinuousActionPool::createFixedActions(const BeliefNode* /*belief*/) const {
	return {};
}

bool ContinuousActionPool::randomiseFixedActions(const BeliefNode* /*belief*/) const {
	return true;
}


/* ---------------------- ChooserDataBase ---------------------- */

void ChooserDataBaseBase::registerDerivedType(const std::string& name, const LoadFromStreamFunction& loader) {
	getDerivedLoadersSingleton()[name] = loader;
}

std::unordered_map<std::string, ChooserDataBaseBase::LoadFromStreamFunction>& ChooserDataBaseBase::getDerivedLoadersSingleton() {
	static std::unordered_map<std::string, ChooserDataBaseBase::LoadFromStreamFunction> singleton;
	return singleton;
}



/* ---------------------- ContinuousActionMap ---------------------- */
ContinuousActionMap::ContinuousActionMap(BeliefNode *owner, ContinuousActionPool *thePool):
        ActionMapping(owner),
        pool(thePool),
        entries(pool->createActionContainer(owner)),
        fixedEntries() {

	auto fixed = pool->createFixedActions(owner);
	for (std::unique_ptr<ContinuousActionConstructionDataBase>& constructionData : fixed) {
		auto& storage = (*entries)[*constructionData];
		if (storage == nullptr) {
		  storage = std::make_unique<ThisActionMapEntry>(this, std::move(constructionData), true);
		}
		fixedEntries.push_back(storage.get());
	}
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


const ContinuousActionMapEntry::ThisActionConstructionData& ContinuousActionMapEntry::getConstructionData() const {
	return *constructionData;
}

/* ------------------- ContinuousActionTextSerializer ------------------- */
void ContinuousActionTextSerializer::saveActionPool( ActionPool const &/*actionPool*/, std::ostream &/*os*/) {
    // Do nothing - the model can create a new one!
}
std::unique_ptr<ActionPool> ContinuousActionTextSerializer::loadActionPool(std::istream &/*is*/) {
    // Use the model to create a new one.
    return getSolver()->getModel()->createActionPool(getSolver());
}

void ContinuousActionTextSerializer::saveActionMapping(ActionMapping const &baseMap, std::ostream &os) {
    ContinuousActionMap const &map = static_cast<ContinuousActionMap const &>(baseMap);


    os << map.getNumberOfVisitedEntries() << " visited actions with ";
    os << map.getNChildren() << " children; ";
    os << map.getTotalVisitCount() << " visits" << std::endl;

    auto entries = map.entries->getEntries();
    os << entries.size() << " entries:" << std::endl;
    for (ActionMappingEntry const* entry : entries) {
    	saveActionMapEntry(static_cast<const ContinuousActionMapEntry&>(*entry), os);
    }

    os << map.fixedEntries.size() << " fixed entries:" << std::endl;
    for (ActionMappingEntry const* baseEntry : map.fixedEntries) {
    	ContinuousActionMapEntry const* entry =  static_cast<ContinuousActionMapEntry const*>(baseEntry);
    	saveConstructionData(entry->getConstructionData(), os);
    }

    if (map.chooserData == nullptr) {
    	os << "chooserData=NULL" << std::endl;
    } else {
    	os << "chooserData:" << std::endl;
    	map.chooserData->saveToStream(map, os);
    }

}

std::unique_ptr<ActionMapping>
ContinuousActionTextSerializer::loadActionMapping(BeliefNode *owner, std::istream &is) {
    std::unique_ptr<ContinuousActionMap> map = std::make_unique<ContinuousActionMap>(owner, static_cast<ContinuousActionPool*>(getSolver()->getActionPool()));
    loadActionMapping(*map, is);
    return std::move(map);
}

void ContinuousActionTextSerializer::loadActionMapping(ContinuousActionMap &map, std::istream &is) {

    std::string line;

    {
    	std::getline(is, line);
    	std::istringstream ss(line);
    	std::string tmpStr;

    	ss >> map.numberOfVisitedEntries >> tmpStr >> tmpStr >> tmpStr;
    	ss >> map.nChildren >> tmpStr;
    	ss >> map.totalVisitCount;
    }

    {
    	std::getline(is, line);
    	std::istringstream ss(line);

    	size_t entrySize;
    	ss >> entrySize;

    	map.entries->clear();

    	for (size_t i=0; i<entrySize; i++) {
    		auto entry = loadActionMapEntry(map, is);
    		std::unique_ptr<ContinuousActionMapEntry>& storage = map.entries->operator[](entry->getConstructionData());
    		if (storage != nullptr) {
    			debug::show_message("Warning: while loading an action map entry, the spot in the container wasn't empty. This is most likely a nasty bug.");
    		}
    		storage = std::move(entry);
    	}
    }

    {
    	std::getline(is, line);
    	std::istringstream ss(line);

    	map.fixedEntries.clear();

    	size_t entrySize;
    	ss >> entrySize;
    	for (size_t i=0; i<entrySize; i++) {
    		auto constructionData = loadConstructionData(is);
    		map.fixedEntries.push_back(map.entries->at(*constructionData).get());
    	}
    }

    {
    	std::getline(is, line);
    	if (line=="chooserData:") {
    		map.chooserData = ChooserDataBaseBase::loadFromStream(map, is);
    	}
    }
}

} /* namespace solver */



