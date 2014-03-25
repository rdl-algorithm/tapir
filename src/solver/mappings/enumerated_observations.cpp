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

#include "ActionMapping.hpp"
#include "ActionPool.hpp"
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
        std::vector<std::unique_ptr<DiscretizedPoint>> observations) :
    observations_(std::move(observations)) {
}

std::unique_ptr<ObservationMapping>
    EnumeratedObservationPool::createObservationMapping() {
    return std::make_unique<EnumeratedObservationMap>(
            actionPool_, observations_);
}


/* ---------------------- EnumeratedObservationMap ---------------------- */
EnumeratedObservationMap::EnumeratedObservationMap(ActionPool *actionPool,
        std::vector<std::unique_ptr<DiscretizedPoint>> const &allObservations) :
                allObservations_(allObservations),
                actionPool_(actionPool),
                children_(allObservations_.size()),
                totalVisitCount_(0) {
}

long EnumeratedObservationMap::size() const {
    return allObservations_.size();
}

BeliefNode* EnumeratedObservationMap::getBelief(
        Observation const &obs) const {
    long code = static_cast<DiscretizedPoint const &>(obs).getBinNumber();
    return children_[code].childNode.get();
}

BeliefNode* EnumeratedObservationMap::createBelief(
        const Observation& obs) {
    long code = static_cast<DiscretizedPoint const &>(obs).getBinNumber();
    children_[code].childNode = std::make_unique<BeliefNode>(actionPool_->createActionMapping());
    return children_[code].childNode.get();
}

void EnumeratedObservationMap::updateVisitCount(Observation const &obs,
        long deltaNVisits) {
    long code = static_cast<DiscretizedPoint const &>(obs).getBinNumber();
    children_[code].visitCount += deltaNVisits;
    totalVisitCount_ += deltaNVisits;
}
long EnumeratedObservationMap::getVisitCount(Observation const &obs) const {
    long code = static_cast<DiscretizedPoint const &>(obs).getBinNumber();
    return children_[code].visitCount;

}
long EnumeratedObservationMap::getTotalVisitCount() const {
    return totalVisitCount_;
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
    return solver_->getModel()->createObservationPool();
}

void EnumeratedObservationTextSerializer::saveObservationMapping(
        ObservationMapping const &map, std::ostream &os) {
    EnumeratedObservationMap const &enumMap = (
            static_cast<EnumeratedObservationMap const &>(map));
    os << enumMap.totalVisitCount_ << " visits {";
    bool isFirst = true;
    for (int i = 0; i < enumMap.size(); i++) {
        EnumeratedObservationMapEntry const &entry = enumMap.children_[i];
        if (entry.childNode != nullptr) {
            if (isFirst) {
                isFirst = false;
            } else {
                os << ", ";
            }
            saveObservation(enumMap.allObservations_[i].get(), os);
            os << ":" << entry.childNode->getId();
            os << " " << entry.visitCount << " v.";
        }
    }
    os << "}";
}

std::unique_ptr<ObservationMapping>
EnumeratedObservationTextSerializer::loadObservationMapping(std::istream &is) {
    std::unique_ptr<ObservationMapping> map(solver_->getObservationPool()->createObservationMapping());
    EnumeratedObservationMap &enumMap = (
                static_cast<EnumeratedObservationMap &>(*map));
    is >> enumMap.totalVisitCount_;
    std::string tmpStr;
    std::getline(is, tmpStr, '{');
    std::getline(is, tmpStr, '}');
    std::istringstream sstr(tmpStr);

    std::string entry;
    while (!sstr.eof()) {
        std::getline(sstr, entry, ',');

        std::istringstream sstr2(entry);
        std::string tmpStr2;
        std::getline(sstr2, tmpStr2, ':');
        std::istringstream sstr3(tmpStr2);
        std::unique_ptr<Observation> obs = loadObservation(sstr3);
        if (obs == nullptr) {
            break;
        }
        long childId;
        sstr2 >> childId;
        long visitCount;
        sstr2 >> visitCount;
        long code = static_cast<DiscretizedPoint const &>(*obs).getBinNumber();
        BeliefNode *node = map->createBelief(*obs);
        solver_->getPolicy()->setNode(childId, node);

        enumMap.children_[code].visitCount = visitCount;
    }
    return std::move(map);
}
} /* namespace solver */



