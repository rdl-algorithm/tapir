#include "discrete_observations_map.hpp"

#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

#include "solver/BeliefNode.hpp"
#include "solver/BeliefTree.hpp"
#include "solver/Model.hpp"

#include "solver/geometry/Observation.hpp"

#include "ActionMapping.hpp"
#include "ActionPool.hpp"
#include "ObservationPool.hpp"
#include "ObservationMapping.hpp"

#include <iostream>
#include <memory>
#include <vector>

#include "global.hpp"

namespace solver {
/* ------------------- ModelWithDiscreteObservations ------------------- */
std::unique_ptr<ObservationPool>
    ModelWithDiscreteObservations::createObservationPool() {
    return std::make_unique<DiscreteObservationPool>();
}


/* --------------------- DiscreteObservationPool --------------------- */
std::unique_ptr<ObservationMapping>
    DiscreteObservationPool::createObservationMapping() {
    return std::make_unique<DiscreteObservationMap>(actionPool_);
}


/* ---------------------- DiscreteObservationMap ---------------------- */
DiscreteObservationMap::DiscreteObservationMap(ActionPool *actionPool) :
                actionPool_(actionPool),
                childMap_() {
}

BeliefNode* DiscreteObservationMap::getBelief(Observation const &obs) const {
    try {
        return childMap_.at(obs.copy()).get();
    } catch (const std::out_of_range &oor) {
        return nullptr;
    }
}

BeliefNode* DiscreteObservationMap::createBelief(const Observation& obs) {
    std::unique_ptr<Observation> observation(obs.copy());
    std::unique_ptr<BeliefNode> node(std::make_unique<BeliefNode>(
            actionPool_->createActionMapping()));
    BeliefNode *nodePtr = node.get();
    childMap_.emplace(std::move(observation), std::move(node));
    return nodePtr;
}


/* ------------------ DiscreteObservationTextSerializer ------------------ */
void DiscreteObservationTextSerializer::saveObservationPool(
        ObservationPool const &/*observationPool*/, std::ostream &/*os*/) {
    // We won't bother writing the pool to file as the model can make a new one.
}

std::unique_ptr<ObservationPool>
DiscreteObservationTextSerializer::loadObservationPool(
        std::istream &/*is*/) {
    // Here we just create a new one.
    return solver_->model_->createObservationPool();
}

void DiscreteObservationTextSerializer::saveObservationMapping(
        ObservationMapping const &map, std::ostream &os) {
    os << "{";
    DiscreteObservationMap const &discMap = (
            static_cast<DiscreteObservationMap const &>(map));
    bool isFirst = true;
    for (DiscreteObservationMap::ChildMap::value_type const &entry : discMap.childMap_) {
        if (isFirst) {
            isFirst = false;
        } else {
            os << ", ";
        }
        saveObservation(entry.first.get(), os);
        os << ":" << entry.second->id_;
    }
    os << "}";
}

std::unique_ptr<ObservationMapping>
DiscreteObservationTextSerializer::loadObservationMapping(std::istream &is) {
    std::unique_ptr<DiscreteObservationMap> map(
                static_cast<DiscreteObservationMap *>(
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



