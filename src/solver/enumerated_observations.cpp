#include "enumerated_observations.hpp"

#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

#include "ActionPool.hpp"
#include "BeliefNode.hpp"
#include "Model.hpp"
#include "ObservationPool.hpp"
#include "ObservationMapping.hpp"

#include <iostream>
#include <memory>
#include <vector>

#include "global.hpp"

namespace solver {
/* ------------------- ModelWithEnumeratedObservations ------------------- */
std::unique_ptr<ObservationPool>
    ModelWithEnumeratedObservations::createObservationPool() {
    return std::make_unique<EnumeratedObservationPool>(getAllObservationsInOrder());
}


/* --------------------- EnumeratedObservationPool --------------------- */
EnumeratedObservationPool::EnumeratedObservationPool(
        std::vector<std::unique_ptr<EnumeratedPoint>> observations) :
    observations_(std::move(observations)) {
}

std::unique_ptr<ObservationMapping>
    EnumeratedObservationPool::createObservationMapping() {
    return std::make_unique<EnumeratedObservationMap>(
            actionPool_, observations_);
}


/* ---------------------- EnumeratedObservationMap ---------------------- */
EnumeratedObservationMap::EnumeratedObservationMap(ActionPool *actionPool,
        std::vector<std::unique_ptr<EnumeratedPoint>> const &observations) :
                actionPool_(actionPool),
                observations_(observations),
                children_(observations_.size()) {
}

long EnumeratedObservationMap::size() const {
    return observations_.size();
}

BeliefNode* EnumeratedObservationMap::getBelief(
        Observation const &obs) const {
    long code = static_cast<EnumeratedPoint const &>(obs).getCode();
    return children_[code].get();
}

BeliefNode* EnumeratedObservationMap::createBelief(
        const Observation& obs) {
    long code = static_cast<EnumeratedPoint const &>(obs).getCode();
    children_[code] = std::make_unique<BeliefNode>(
            actionPool_->createActionMapping());
    return children_[code].get();
}


/* ------------------- EnumeratedObservationTextSerializer ------------------- */
void EnumeratedObservationTextSerializer::saveObservationPool(
        ObservationPool const &observationPool, std::ostream &os) {
    // We won't bother writing the pool to file as the model can make a new one.
}
std::unique_ptr<ObservationPool>
EnumeratedObservationTextSerializer::loadObservationPool(
        std::istream &is) {
    // Here we just create a new one.
    return solver_->model_->createObservationPool();
}

void EnumeratedObservationTextSerializer::saveObservationMapping(
        ObservationMapping const &map, std::ostream &os) {
    os << "{";
    EnumeratedObservationMap const &enumMap = (
            static_cast<EnumeratedObservationMap const &>(map));
    for (int i = 0; i < enumMap.size(); i++) {
        BeliefNode *child = enumMap.children_[i].get();
        if (child != nullptr) {
            saveObservation(observations_[i]);
            os << ":" << child->id_;
            if (i != enumMap.size() - 1) {
                os << ", ";
            }
        }
    }
    os << "}";
}

std::unique_ptr<ObservationMapping>
EnumeratedObservationTextSerializer::loadObservationMapping(std::istream &is) {
    std::unique_ptr<EnumeratedObservationMap> map(
            solver_->observationPool_->createObservationMapping());
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



