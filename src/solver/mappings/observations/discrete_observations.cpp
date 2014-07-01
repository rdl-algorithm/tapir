/** file: discrete_observations.cpp
 *
 * Contains the implementations of the classes for discrete observation mappings.
 */
#include "solver/mappings/observations/discrete_observations.hpp"

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <sstream>
#include <vector>

#include "global.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/BeliefTree.hpp"
#include "solver/abstract-problem/Model.hpp"

#include "solver/abstract-problem/Observation.hpp"

#include "solver/mappings/observations/ObservationPool.hpp"
#include "solver/mappings/observations/ObservationMapping.hpp"

namespace solver {
/* --------------------- DiscreteObservationPool --------------------- */
std::unique_ptr<ObservationMapping> DiscreteObservationPool::createObservationMapping(ActionNode *owner) {
    return std::make_unique<DiscreteObservationMap>(owner);
}

/* ---------------------- DiscreteObservationMap ---------------------- */
DiscreteObservationMap::DiscreteObservationMap(ActionNode *owner) :
        ObservationMapping(owner),
        childMap_(),
        totalVisitCount_(0) {
}

BeliefNode* DiscreteObservationMap::getBelief(Observation const &obs) const {
    ObservationMappingEntry const *entry = getEntry(obs);
    if (entry == nullptr) {
        return nullptr;
    } else {
        return entry->getBeliefNode();
    }
}
BeliefNode* DiscreteObservationMap::createBelief(const Observation& obs) {
    std::unique_ptr<DiscreteObservationMapEntry> entry = (
          std::make_unique<DiscreteObservationMapEntry>());
    entry->map_ = this;
    entry->observation_ = obs.copy();
    entry->childNode_ = std::make_unique<BeliefNode>(entry.get());
    BeliefNode *node = entry->childNode_.get();

    childMap_.emplace(obs.copy(), std::move(entry));
    return node;
}
long DiscreteObservationMap::getNChildren() const {
    return childMap_.size();
}

ObservationMappingEntry *DiscreteObservationMap::getEntry(Observation const &obs) {
    try {
        return childMap_.at(obs.copy()).get();
    } catch (const std::out_of_range &oor) {
        return nullptr;
    }
}
ObservationMappingEntry const *DiscreteObservationMap::getEntry(Observation const &obs) const {
    try {
        return childMap_.at(obs.copy()).get();
    } catch (const std::out_of_range &oor) {
        return nullptr;
    }
}
std::vector<ObservationMappingEntry const *> DiscreteObservationMap::getAllEntries() const {
    std::vector<ObservationMappingEntry const *> returnEntries;
    for (ChildMap::value_type const &mapEntry : childMap_) {
        returnEntries.push_back(mapEntry.second.get());
    }
    return returnEntries;
}

long DiscreteObservationMap::getTotalVisitCount() const {
    return totalVisitCount_;
}

/* ----------------- DiscreteObservationMapEntry ----------------- */
ObservationMapping *DiscreteObservationMapEntry::getMapping() const {
    return map_;
}
std::unique_ptr<Observation> DiscreteObservationMapEntry::getObservation() const {
    return observation_->copy();
}
BeliefNode *DiscreteObservationMapEntry::getBeliefNode() const {
    return childNode_.get();
}
long DiscreteObservationMapEntry::getVisitCount() const {
    return visitCount_;
}

void DiscreteObservationMapEntry::updateVisitCount(long deltaNVisits) {
    visitCount_ += deltaNVisits;
    map_->totalVisitCount_ += deltaNVisits;
}

/* ------------------ DiscreteObservationTextSerializer ------------------ */
void DiscreteObservationTextSerializer::saveObservationPool(
        ObservationPool const &/*observationPool*/, std::ostream &/*os*/) {
    // We won't bother writing the pool to file as the model can make a new one.
}

std::unique_ptr<ObservationPool> DiscreteObservationTextSerializer::loadObservationPool(
        std::istream &/*is*/) {
    // Here we just create a new one.
    return getSolver()->getModel()->createObservationPool(getSolver());
}

void DiscreteObservationTextSerializer::saveObservationMapping(
        ObservationMapping const &map, std::ostream &os) {
    DiscreteObservationMap const &discMap =
            (static_cast<DiscreteObservationMap const &>(map));
    os << discMap.getNChildren() << " observation children; ";
    os << discMap.getTotalVisitCount() << " visits {" << std::endl;
    std::vector<std::string> lines;
    for (DiscreteObservationMap::ChildMap::value_type const &entry : discMap.childMap_) {
        std::ostringstream sstr;
        sstr << "\t";
        saveObservation(entry.first.get(), sstr);
        sstr << " -> NODE " << entry.second->childNode_->getId();
        sstr << "; " << entry.second->visitCount_ << " visits";
        sstr << std::endl;
        lines.push_back(sstr.str());
    }
    std::sort(lines.begin(), lines.end());
    for (std::string line : lines) {
        os << line;
    }
    os << "}" << std::endl;
}

std::unique_ptr<ObservationMapping> DiscreteObservationTextSerializer::loadObservationMapping(
        ActionNode *owner, std::istream &is) {
    std::unique_ptr<ObservationMapping> map(
            getSolver()->getObservationPool()->createObservationMapping(owner));
    DiscreteObservationMap &discMap =
                    (static_cast<DiscreteObservationMap &>(*map));
    std::string line;
    std::getline(is, line);
    std::string tmpStr;
    std::istringstream totalsStream(line);
    long nChildren;
    totalsStream >> nChildren >> tmpStr >> tmpStr;
    totalsStream >> discMap.totalVisitCount_;

    for (int i = 0; i < nChildren; i++) {
        std::getline(is, line);
        std::istringstream entryStream(line);
        std::unique_ptr<Observation> obs = loadObservation(entryStream);

        entryStream >> tmpStr >> tmpStr;
        std::getline(entryStream, tmpStr, ';');
        long childId;
        std::istringstream(tmpStr) >> childId;
        long visitCount;
        entryStream >> visitCount;

        // Create the mapping entry and set its values.
        std::unique_ptr<DiscreteObservationMapEntry> entry = (
                    std::make_unique<DiscreteObservationMapEntry>());
        entry->map_ = &discMap;
        entry->observation_ = std::move(obs);
        entry->childNode_ = std::make_unique<BeliefNode>(childId, entry.get());
        entry->visitCount_ = visitCount;

        // Add the entry to the map
        discMap.childMap_.emplace(entry->observation_->copy(), std::move(entry));
    }
    // Read the last line for the closing brace.
    std::getline(is, line);
    return std::move(map);
}
} /* namespace solver */

