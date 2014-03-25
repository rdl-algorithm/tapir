#include "approximate_observations.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/BeliefTree.hpp"
#include "solver/abstract-problem/Model.hpp"
#include "solver/abstract-problem/Observation.hpp"

#include "ActionMapping.hpp"
#include "ActionPool.hpp"
#include "ObservationPool.hpp"
#include "ObservationMapping.hpp"

#include <memory>
#include <utility>
#include <vector>

#include "global.hpp"

namespace solver {
/* ------------------- ModelWithApproximateObservations ------------------- */
std::unique_ptr<ObservationPool>
    ModelWithApproximateObservations::createObservationPool() {
    return std::make_unique<ApproximateObservationPool>(
            getMaxObservationDistance());
}

/* --------------------- ApproximateObservationPool --------------------- */
ApproximateObservationPool::ApproximateObservationPool(double maxDistance) :
        maxDistance_(maxDistance) {
}

std::unique_ptr<ObservationMapping>
    ApproximateObservationPool::createObservationMapping() {
    return std::make_unique<ApproximateObservationMap>(actionPool_,
            maxDistance_);
}

/* ---------------------- ApproximateObservationMap ---------------------- */
ApproximateObservationMap::ApproximateObservationMap(ActionPool *actionPool,
        double maxDistance) :
                actionPool_(actionPool),
                maxDistance_(maxDistance),
                children_(),
                totalVisitCount_(0) {
}

BeliefNode* ApproximateObservationMap::getBelief(
        Observation const &obs) const {
    ApproximateObservationMapEntry const *entry = getEntry(obs);
    if (entry == nullptr) {
        return nullptr;
    }
    return entry->childNode.get();
}

BeliefNode* ApproximateObservationMap::createBelief(
        const Observation& obs) {
    ApproximateObservationMapEntry entry;
    entry.observation = obs.copy();
    entry.childNode = std::make_unique<BeliefNode>(actionPool_->createActionMapping());
    BeliefNode *node = entry.childNode.get();
    children_.push_back(std::move(entry));
    return node;
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
std::unique_ptr<ObservationPool>
ApproximateObservationTextSerializer::loadObservationPool(
        std::istream &/*is*/) {
    // Here we just create a new one.
    return solver_->getModel()->createObservationPool();
}

void ApproximateObservationTextSerializer::saveObservationMapping(
        ObservationMapping const &map, std::ostream &os) {
    ApproximateObservationMap const &approxMap = (
                static_cast<ApproximateObservationMap const &>(map));
    os << approxMap.totalVisitCount_ << " visits {";
    bool isFirst = true;
    for (ApproximateObservationMapEntry const &entry : approxMap.children_) {
        if (isFirst) {
            isFirst = false;
        } else {
            os << ", ";
        }
        saveObservation(entry.observation.get(), os);
        os << ":" << entry.childNode->getId();
        os << " " << entry.visitCount << " v.";
    }
    os << "}";
}

std::unique_ptr<ObservationMapping>
ApproximateObservationTextSerializer::loadObservationMapping(std::istream &is) {
    std::unique_ptr<ObservationMapping> map(
            solver_->getObservationPool()->createObservationMapping());
    ApproximateObservationMap &approxMap = (
                static_cast<ApproximateObservationMap &>(*map));
    is >> approxMap.totalVisitCount_;
    std::string tmpStr;
    std::getline(is, tmpStr, '{');
    std::getline(is, tmpStr, '}');
    std::istringstream sstr(tmpStr);

    std::string entryString;
    while (!sstr.eof()) {
        std::getline(sstr, entryString, ',');
        std::istringstream sstr2(entryString);
        std::unique_ptr<Observation> obs = loadObservation(sstr2);
        std::string tmpStr2;
        sstr2 >> tmpStr2;

        if (obs == nullptr) {
            break;
        }
        long childId;
        sstr2 >> childId;
        ApproximateObservationMapEntry entry;
        sstr2 >> entry.visitCount;
        entry.observation = std::move(obs);
        entry.childNode = std::make_unique<BeliefNode>(
                approxMap.actionPool_->createActionMapping());

        BeliefNode *node = entry.childNode.get();
        solver_->getPolicy()->setNode(childId, node);
    }
    return std::move(map);
}
} /* namespace solver */



