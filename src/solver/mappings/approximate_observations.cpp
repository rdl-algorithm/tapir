#include "approximate_observations.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/BeliefTree.hpp"
#include "solver/Model.hpp"
#include "solver/geometry/Observation.hpp"

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
                children_() {
}

BeliefNode* ApproximateObservationMap::getBelief(
        Observation const &obs) const {
    double shortestDistance = maxDistance_;
    BeliefNode *bestNode = nullptr;
    for (Entry const &entry : children_) {
        double distance = entry.first->distanceTo(obs);
        if (distance <= shortestDistance) {
            shortestDistance = distance;
            bestNode = entry.second.get();
        }
    }
    return bestNode;
}

BeliefNode* ApproximateObservationMap::createBelief(
        const Observation& obs) {
    Entry val = std::make_pair(obs.copy(), std::make_unique<BeliefNode>(
            actionPool_->createActionMapping()));
    BeliefNode *node = val.second.get();
    children_.push_back(std::move(val));
    return node;
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
    return solver_->model_->createObservationPool();
}

void ApproximateObservationTextSerializer::saveObservationMapping(
        ObservationMapping const &map, std::ostream &os) {
    os << "{";
    ApproximateObservationMap const &approxMap = (
            static_cast<ApproximateObservationMap const &>(map));
    for (ApproximateObservationMap::ChildMappingVector::const_iterator
            it = approxMap.children_.cbegin();
            it != approxMap.children_.cend(); it++) {
        saveObservation(it->first.get(), os);
        os << ":" << it->second->id_;
        if (std::next(it) != approxMap.children_.cend()) {
            os << ", ";
        }
    }
     os << "}";
}

std::unique_ptr<ObservationMapping>
ApproximateObservationTextSerializer::loadObservationMapping(std::istream &is) {
    std::unique_ptr<ApproximateObservationMap> map(
            static_cast<ApproximateObservationMap *>(
            solver_->observationPool_->createObservationMapping().release()));
    std::string tmpStr;
    std::getline(is, tmpStr, '{');
    std::getline(is, tmpStr, '}');
    std::stringstream sstr(tmpStr);

    std::string entry;
    while (!sstr.eof()) {
        std::getline(sstr, entry, ',');

        std::stringstream sstr2(entry);
        std::string tmpStr2;
        std::getline(sstr2, tmpStr2, ':');
        std::stringstream sstr3(tmpStr2);
        std::unique_ptr<Observation> obs = loadObservation(sstr3);
        if (obs == nullptr) {
            break;
        }
        long childId;
        sstr2 >> childId;
        if (solver_->policy_->allNodes_[childId] != nullptr) {
            std::cerr << "ERROR: Node already present !?" << std::endl;
        }
        BeliefNode *node = map->createBelief(*obs);
        node->id_ = childId;
        solver_->policy_->allNodes_[childId] = node;
    }
    return std::move(map);
}
} /* namespace solver */



