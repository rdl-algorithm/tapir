#include "enumerated_observations.hpp"

#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

#include "solver/BeliefNode.hpp"
#include "solver/BeliefTree.hpp"
#include "solver/abstract-problem/Model.hpp"

#include "solver/abstract-problem/DiscretizedPoint.hpp"
#include "solver/abstract-problem/Observation.hpp"

#include "ObservationPool.hpp"
#include "ObservationMapping.hpp"

#include <iostream>
#include <memory>
#include <vector>

#include "global.hpp"

namespace solver {
/* --------------------- EnumeratedObservationPool --------------------- */
EnumeratedObservationPool::EnumeratedObservationPool(
        std::vector<std::unique_ptr<DiscretizedPoint>> observations) :
                observations_(std::move(observations)) {
}

std::unique_ptr<ObservationMapping>
    EnumeratedObservationPool::createObservationMapping() {
    return std::make_unique<EnumeratedObservationMap>(observations_);
}


/* ---------------------- EnumeratedObservationMap ---------------------- */
EnumeratedObservationMap::EnumeratedObservationMap(
        std::vector<std::unique_ptr<DiscretizedPoint>> const &allObservations) :
                owningActionNode_(nullptr),
                allObservations_(allObservations),
                children_(allObservations_.size()),
                nChildren_(0),
                totalVisitCount_(0) {
}

void EnumeratedObservationMap::setOwner(ActionNode *owner) {
    owningActionNode_ = owner;
}
ActionNode *EnumeratedObservationMap::getOwner() const {
    return owningActionNode_;
}


BeliefNode* EnumeratedObservationMap::getBelief(
        Observation const &obs) const {
    long code = static_cast<DiscretizedPoint const &>(obs).getBinNumber();
    EnumeratedObservationMapEntry *entry = children_[code].get();
    if (entry == nullptr) {
        return nullptr;
    }
    return entry->getBeliefNode();
}
BeliefNode* EnumeratedObservationMap::createBelief(
        const Observation& obs) {
    long code = static_cast<DiscretizedPoint const &>(obs).getBinNumber();
    std::unique_ptr<EnumeratedObservationMapEntry> entry = (
            std::make_unique<EnumeratedObservationMapEntry>());
    entry->map_ = this;
    entry->index_ = code;
    entry->childNode_ = std::make_unique<BeliefNode>(entry.get());
    BeliefNode *node = entry->childNode_.get();

    children_[code] = std::move(entry);
    nChildren_++;
    return node;
}

long EnumeratedObservationMap::getNChildren() const {
    return nChildren_;
}
ObservationMappingEntry *EnumeratedObservationMap::getEntry(Observation const &obs) {
    long code = static_cast<DiscretizedPoint const &>(obs).getBinNumber();
    return children_[code].get();
}
ObservationMappingEntry const*EnumeratedObservationMap::getEntry(Observation const &obs) const {
    long code = static_cast<DiscretizedPoint const &>(obs).getBinNumber();
    return children_[code].get();
}
std::vector<ObservationMappingEntry const *> EnumeratedObservationMap::getAllEntries() const {
    std::vector<ObservationMappingEntry const *> returnEntries;
    for (std::unique_ptr<EnumeratedObservationMapEntry> const &entry : children_) {
        if (entry != nullptr) {
            returnEntries.push_back(entry.get());
        }
    }
    return returnEntries;
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
    for (std::unique_ptr<EnumeratedObservationMapEntry> const &entry : enumMap.children_) {
        if (entry != nullptr) {
            os << "\t";
            saveObservation(entry->getObservation().get(), os);
            os << " -> NODE " << entry->childNode_->getId();
            os << "; " << entry->visitCount_ << " visits";
            os << std::endl;
        }
    }
    os << "}" << std::endl;
}

std::unique_ptr<ObservationMapping>
EnumeratedObservationTextSerializer::loadObservationMapping(
        std::istream &is) {
    std::unique_ptr<ObservationMapping> map(getSolver()->getObservationPool()->createObservationMapping());
    EnumeratedObservationMap &enumMap = (
                static_cast<EnumeratedObservationMap &>(*map));
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

        // Create the child node and set its values correctly.
        long code = static_cast<DiscretizedPoint const &>(*obs).getBinNumber();

        std::unique_ptr<EnumeratedObservationMapEntry> entry = (
                std::make_unique<EnumeratedObservationMapEntry>());
        entry->map_ = &enumMap;
        entry->index_ = code;
        entry->childNode_ = std::make_unique<BeliefNode>(childId, entry.get());
        entry->visitCount_ = visitCount;

        // Add the entry to the map.
        enumMap.children_[code] = std::move(entry);
        enumMap.nChildren_++;
    }
    // Read the last line for the closing brace.
    std::getline(is, line);
    return std::move(map);
}
} /* namespace solver */



