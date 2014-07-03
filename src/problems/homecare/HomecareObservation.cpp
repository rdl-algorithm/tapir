#include "HomecareObservation.hpp"

#include <cstddef>                      // for size_t

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/abstract-problem/Observation.hpp"             // for Observation

#include "HomecareModel.hpp"

namespace homecare {
HomecareObservation::HomecareObservation(GridPosition robotPos,
    GridPosition targetPos, int targetRegion, bool call) :
        robotPos_(robotPos),
        targetPos_(targetPos),
        targetRegion_(targetRegion),
        call_(call) {
}
std::unique_ptr<solver::Observation>
HomecareObservation::copy() const {
    return std::make_unique<HomecareObservation>(robotPos_, targetPos_,
        targetRegion_, call_);
}

double HomecareObservation::distanceTo(
        solver::Observation const &otherObs) const {
    HomecareObservation const &other =
            static_cast<HomecareObservation const &>(otherObs);
    double dist = 0;
    dist += targetPos_ == other.targetPos_ ? 0 : 1;
    dist += targetRegion_ == other.targetRegion_ ? 0 : 1;
    dist += call_ == other.call_ ? 0 : 1; 
    return dist;
}

bool HomecareObservation::equals(
        solver::Observation const &otherObs) const {
    HomecareObservation const &other =
        static_cast<HomecareObservation const &>(otherObs);
    return robotPos_ == other.robotPos_ && targetPos_ == other.targetPos_ &&
        targetRegion_ == other.targetRegion_ && call_ == other.call_;
}

std::size_t HomecareObservation::hash() const {
    std::size_t hashValue = 0;
    abt::hash_combine(hashValue, robotPos_.i);
    abt::hash_combine(hashValue, robotPos_.j);
    abt::hash_combine(hashValue, targetPos_.i);
    abt::hash_combine(hashValue, targetPos_.j);
    abt::hash_combine(hashValue, targetRegion_);
    abt::hash_combine(hashValue, call_);
    return hashValue;
}

void HomecareObservation::print(std::ostream &os) const {
    os << robotPos_ << ", ";
    os << targetPos_ << ", ";
    os << "target region: " << targetRegion_ << ", ";
    if (call_) {
        os << "calling";
    } else {
        os << "not calling";
    }
} 

GridPosition HomecareObservation::getRobotPos() const {
    return robotPos_;
}

GridPosition HomecareObservation::getTargetPos() const {
    return targetPos_;
}

int HomecareObservation::getTargetRegion() const {
    return targetRegion_;
}

bool HomecareObservation::getCall() const {
    return call_;
}
}
/* namespace homecare */
