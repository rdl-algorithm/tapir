/** @file enumerated_observations.cpp
 *
 * Contains the implementations of the classes for enumerated observation mappings.
 */
#include "solver/mappings/observations/enumerated_observations.hpp"

#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

#include "global.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/BeliefTree.hpp"
#include "solver/abstract-problem/Model.hpp"

#include "solver/abstract-problem/DiscretizedPoint.hpp"
#include "solver/abstract-problem/Observation.hpp"

#include "solver/mappings/observations/ObservationPool.hpp"
#include "solver/mappings/observations/ObservationMapping.hpp"

namespace solver {
/* --------------------- EnumeratedObservationPool --------------------- */
EnumeratedObservationPool::EnumeratedObservationPool(
        std::vector<std::unique_ptr<DiscretizedPoint>> observations) :
                observations_(std::move(observations)) {
}

std::unique_ptr<ObservationMapping>
    EnumeratedObservationPool::createObservationMapping(ActionNode *owner) {
    return std::make_unique<EnumeratedObservationMap>(owner, observations_);
}


/* ---------------------- EnumeratedObservationMap ---------------------- */
EnumeratedObservationMap::EnumeratedObservationMap(ActionNode *owner,
        std::vector<std::unique_ptr<DiscretizedPoint>> const &allObservations) :
                ObservationMapping(owner),
                allObservations_(allObservations),
                nObservations_(allObservations.size()),
                entries_(std::make_unique<EnumeratedObservationMapEntry[]>(nObservations_)),
                nChildren_(0),
                totalVisitCount_(0) {
    for (int i = 0; i < nObservations_; i++) {
        entries_[i].map_ = this;
        entries_[i].index_ = i;
        entries_[i].visitCount_ = 0;
    }
}


BeliefNode* EnumeratedObservationMap::getBelief(
        Observation const &obs) const {
    long code = static_cast<DiscretizedPoint const &>(obs).getBinNumber();
    return entries_[code].getBeliefNode();
}
BeliefNode* EnumeratedObservationMap::createBelief(
        const Observation& obs) {
    long code = static_cast<DiscretizedPoint const &>(obs).getBinNumber();
    EnumeratedObservationMapEntry &entry = entries_[code];
    entry.childNode_ = std::make_unique<BeliefNode>(&entry);
    nChildren_++;
    return entry.getBeliefNode();
}
long EnumeratedObservationMap::getNChildren() const {
    return nChildren_;
}

ObservationMappingEntry *EnumeratedObservationMap::getEntry(Observation const &obs) {
    long code = static_cast<DiscretizedPoint const &>(obs).getBinNumber();
    return &entries_[code];
}
ObservationMappingEntry const *EnumeratedObservationMap::getEntry(Observation const &obs) const {
    long code = static_cast<DiscretizedPoint const &>(obs).getBinNumber();
    return &entries_[code];
}
std::vector<ObservationMappingEntry const *> EnumeratedObservationMap::getAllEntries() const {
    std::vector<ObservationMappingEntry const *> returnEntries;
    for (int i = 0; i < nObservations_; i++) {
        if (entries_[i].childNode_ != nullptr) {
            returnEntries.push_back(&entries_[i]);
        }
    }
    return returnEntries;
}

virtual void EnumeratedObservationMap::deleteEntry(ObservationMappingEntry const *entry) {
    static_cast<EnumeratedObservationMapEntry const &>(*entry).childNode_ = nullptr;
}

long EnumeratedObservationMap::getTotalVisitCount() const {
    return totalVisitCount_;
}

/* ----------------- EnumeratedObservationMapEntry ----------------- */
ObservationMapping *EnumeratedObservationMapEntry::getMapping() const {
    return map_;
}
std::unique_ptr<Observation> EnumeratedObservationMapEntry::getObservation() const {
    EnumeratedObservationMap const &enumMap = static_cast<EnumeratedObservationMap const &>(*map_);
    return enumMap.allObservations_[index_]->copy();
}
BeliefNode *EnumeratedObservationMapEntry::getBeliefNode() const {
    return childNode_.get();
}
long EnumeratedObservationMapEntry::getVisitCount() const {
    return visitCount_;
}

void EnumeratedObservationMapEntry::updateVisitCount(long deltaNVisits) {
    visitCount_ += deltaNVisits;
    map_->totalVisitCount_ += deltaNVisits;
}


/* ------------------ EnumeratedObservationTextSerializer ------------------ */
void EnumeratedObservationTextSerializer::saveObservationPool(
        ObservationPool const &/*observationPool*/, std::ostream &/*os*/) {
    // We won't bother writing the pool to file as the model can make a new one.
}

std::unique_ptr<ObservationPool>
EnumeratedObservationTextSerializer::loadObservationPool(
        std::istream &/*is*/) {
    // Here we just create a new one.
    return getSolver()->getModel()->createObservationPool(getSolver());
}

void EnumeratedObservationTextSerializer::saveObservationMapping(
        ObservationMapping const &map, std::ostream &os) {
    EnumeratedObservationMap const &enumMap = (
            static_cast<EnumeratedObservationMap const &>(map));
    os << enumMap.getNChildren() << " observation children; ";
    os << enumMap.getTotalVisitCount() << " visits {" << std::endl;
    for (int i = 0; i < enumMap.nObservations_; i++) {
        EnumeratedObservationMapEntry const &entry = enumMap.entries_[i];
        if (entry.childNode_ != nullptr) {
            os << "\t";
            saveObservation(entry.getObservation().get(), os);
            os << " -> NODE " << entry.childNode_->getId();
            os << "; " << entry.visitCount_ << " visits";
            os << std::endl;
        }
    }
    os << "}" << std::endl;
}

std::unique_ptr<ObservationMapping>
EnumeratedObservationTextSerializer::loadObservationMapping(ActionNode *owner,
        std::istream &is) {
    std::unique_ptr<ObservationMapping> map(
getSolver()->getObservationPool()->createObservationMapping(owner));
    EnumeratedObservationMap &enumMap = static_cast<EnumeratedObservationMap &>(*map);
    std::string line;
    std::getline(is, line);
    std::string tmpStr;
    std::istringstream totalsStream(line);
    long nChildren;
    totalsStream >> nChildren >> tmpStr >> tmpStr;
    totalsStream >> enumMap.totalVisitCount_;

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

        // Create the child node for the entry and set its visit count.
        long code = static_cast<DiscretizedPoint const &>(*obs).getBinNumber();
        EnumeratedObservationMapEntry &entry = enumMap.entries_[code];
        entry.childNode_ = std::make_unique<BeliefNode>(childId, &entry);
        enumMap.nChildren_++;
        entry.visitCount_ = visitCount;
    }
    // Read the last line for the closing brace.
    std::getline(is, line);
    return std::move(map);
}
} /* namespace solver */



