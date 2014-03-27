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
                nChildren_(0),
                totalVisitCount_(0) {
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
    nChildren_++;
    return children_[code].childNode.get();
}


long EnumeratedObservationMap::getNChildren() const {
    return nChildren_;
}
long EnumeratedObservationMap::size() const {
    return allObservations_.size();
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
    os << enumMap.getNChildren() << " observation children; ";
    os << enumMap.getTotalVisitCount() << " visits {" << std::endl;
    for (int i = 0; i < enumMap.size(); i++) {
        EnumeratedObservationMapEntry const &entry = enumMap.children_[i];
        if (entry.childNode != nullptr) {
            os << "\t";
            saveObservation(enumMap.allObservations_[i].get(), os);
            os << " -> NODE " << entry.childNode->getId();
            os << "; " << entry.visitCount << " visits";
            os << std::endl;
        }
    }
    os << "}" << std::endl;
}

std::unique_ptr<ObservationMapping>
EnumeratedObservationTextSerializer::loadObservationMapping(std::istream &is) {
    std::unique_ptr<ObservationMapping> map(solver_->getObservationPool()->createObservationMapping());
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
        BeliefNode *node = map->createBelief(*obs);
        solver_->getPolicy()->setNode(childId, node);
        enumMap.children_[code].visitCount = visitCount;
    }
    // Read the last line for the closing brace.
    std::getline(is, line);
    return std::move(map);
}
} /* namespace solver */



