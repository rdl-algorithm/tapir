#include "approximate_observations.hpp"

#include <algorithm>
#include <memory>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "solver/BeliefNode.hpp"
#include "solver/BeliefTree.hpp"
#include "solver/abstract-problem/Model.hpp"
#include "solver/abstract-problem/Observation.hpp"

#include "ActionMapping.hpp"
#include "ActionPool.hpp"
#include "ObservationPool.hpp"
#include "ObservationMapping.hpp"

#include "global.hpp"

namespace solver {
/* ------------------- ModelWithApproximateObservations ------------------- */
std::unique_ptr<ObservationPool> ModelWithApproximateObservations::createObservationPool() {
    return std::make_unique<ApproximateObservationPool>(
            getMaxObservationDistance());
}

/* --------------------- ApproximateObservationPool --------------------- */
ApproximateObservationPool::ApproximateObservationPool(double maxDistance) :
        maxDistance_(maxDistance) {
}

std::unique_ptr<ObservationMapping> ApproximateObservationPool::createObservationMapping() {
    return std::make_unique<ApproximateObservationMap>(actionPool_,
            maxDistance_);
}

/* ---------------------- ApproximateObservationMap ---------------------- */
ApproximateObservationMap::ApproximateObservationMap(ActionPool *actionPool,
        double maxDistance) :
        actionPool_(actionPool), maxDistance_(maxDistance), children_(), totalVisitCount_(
                0) {
}

BeliefNode* ApproximateObservationMap::getBelief(Observation const &obs) const {
    ApproximateObservationMapEntry const *entry = getEntry(obs);
    if (entry == nullptr) {
        return nullptr;
    }
    return entry->childNode.get();
}

BeliefNode* ApproximateObservationMap::createBelief(const Observation& obs) {
    ApproximateObservationMapEntry entry;
    entry.observation = obs.copy();
    entry.childNode = std::make_unique<BeliefNode>(
            actionPool_->createActionMapping());
    BeliefNode *node = entry.childNode.get();
    children_.push_back(std::move(entry));
    return node;
}

long ApproximateObservationMap::getNChildren() const {
    return children_.size();
}

void ApproximateObservationMap::updateVisitCount(Observation const &obs,
        long deltaNVisits) {
    getEntry(obs)->visitCount += deltaNVisits;
    totalVisitCount_ += deltaNVisits;
}
long ApproximateObservationMap::getVisitCount(Observation const &obs) const {
    return getEntry(obs)->visitCount;
}
long ApproximateObservationMap::getTotalVisitCount() const {
    return totalVisitCount_;
}

ApproximateObservationMapEntry const *ApproximateObservationMap::getEntry(
        Observation const &obs) const {
    double shortestDistance = maxDistance_;
    ApproximateObservationMapEntry const *bestEntry = nullptr;
    for (ApproximateObservationMapEntry const &entry : children_) {
        double distance = entry.observation->distanceTo(obs);
        if (distance <= shortestDistance) {
            shortestDistance = distance;
            bestEntry = &entry;
        }
    }
    return bestEntry;
}

ApproximateObservationMapEntry *ApproximateObservationMap::getEntry(
        Observation const &obs) {
    double shortestDistance = maxDistance_;
    ApproximateObservationMapEntry *bestEntry = nullptr;
    for (ApproximateObservationMapEntry &entry : children_) {
        double distance = entry.observation->distanceTo(obs);
        if (distance <= shortestDistance) {
            shortestDistance = distance;
            bestEntry = &entry;
        }
    }
    return bestEntry;
}

/* ----------------- ApproximateObservationTextSerializer ----------------- */
void ApproximateObservationTextSerializer::saveObservationPool(
        ObservationPool const &/*observationPool*/, std::ostream &/*os*/) {
    // We won't bother writing the pool to file as the model can make a new one.
}
std::unique_ptr<ObservationPool> ApproximateObservationTextSerializer::loadObservationPool(
        std::istream &/*is*/) {
    // Here we just create a new one.
    return solver_->getModel()->createObservationPool();
}

void ApproximateObservationTextSerializer::saveObservationMapping(
        ObservationMapping const &map, std::ostream &os) {
    ApproximateObservationMap const &approxMap =
            (static_cast<ApproximateObservationMap const &>(map));
    os << approxMap.getNChildren() << " observation children; ";
    os << approxMap.getTotalVisitCount() << " visits {" << std::endl;
    std::vector<std::string> lines;
    for (ApproximateObservationMapEntry const &entry : approxMap.children_) {
        std::ostringstream sstr;
        sstr << "\t";
        saveObservation(entry.observation.get(), sstr);
        sstr << " -> NODE " << entry.childNode->getId();
        sstr << "; " << entry.visitCount << " visits";
        sstr << std::endl;
        lines.push_back(sstr.str());
    }
    // Sort the lines so that I/O is 1:1
    std::sort(lines.begin(), lines.end());
    for (std::string line : lines) {
        os << line;
    }
    os << "}" << std::endl;
}

std::unique_ptr<ObservationMapping> ApproximateObservationTextSerializer::loadObservationMapping(
        std::istream &is) {
    std::unique_ptr<ObservationMapping> map(
            solver_->getObservationPool()->createObservationMapping());
    ApproximateObservationMap &approxMap =
            (static_cast<ApproximateObservationMap &>(*map));
    std::string line;
    std::getline(is, line);
    std::string tmpStr;
    std::istringstream totalsStream(line);
    long nChildren;
    totalsStream >> nChildren >> tmpStr >> tmpStr;
    totalsStream >> approxMap.totalVisitCount_;

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

        // Create the entry and set its values correctly.
        ApproximateObservationMapEntry entry;
        entry.visitCount = visitCount;
        entry.observation = std::move(obs);
        entry.childNode = std::make_unique<BeliefNode>(
                approxMap.actionPool_->createActionMapping());

        // Add the node to the tree index
        BeliefNode *node = entry.childNode.get();
        solver_->getPolicy()->setNode(childId, node);

        // Add the entry to the vector
        approxMap.children_.push_back(std::move(entry));
    }
    // Read the last line for the closing brace.
    std::getline(is, line);
    return std::move(map);
}
} /* namespace solver */

