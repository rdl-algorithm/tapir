#include "TrackerObservation.hpp"

#include <cstddef>                      // for size_t

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/abstract-problem/Observation.hpp"             // for Observation

#include "TrackerModel.hpp"

namespace tracker {
TrackerObservation::TrackerObservation(GridPosition robotPos, int robotYaw,
            bool _seesTarget) :
                    robotPos_(robotPos),
                    robotYaw_(robotYaw),
                    seesTarget_(_seesTarget) {
}
std::unique_ptr<solver::Observation>
TrackerObservation::copy() const {
    return std::make_unique<TrackerObservation>(robotPos_, robotYaw_, seesTarget_);
}

double TrackerObservation::distanceTo(
        solver::Observation const &otherObs) const {
    TrackerObservation const &other =
            static_cast<TrackerObservation const &>(otherObs);
    return seesTarget_ == other.seesTarget_ ? 0 : 1;
}

bool TrackerObservation::equals(
        solver::Observation const &otherObs) const {
    TrackerObservation const &other =
        static_cast<TrackerObservation const &>(otherObs);
    return robotPos_ == other.robotPos_ && robotYaw_ == other.robotYaw_ &&
        seesTarget_ == other.seesTarget_;
}

std::size_t TrackerObservation::hash() const {
    std::size_t hashValue = 0;
    abt::hash_combine(hashValue, robotPos_.i);
    abt::hash_combine(hashValue, robotPos_.j);
    abt::hash_combine(hashValue, robotYaw_);
    abt::hash_combine(hashValue, seesTarget_);
    return hashValue;
}

void TrackerObservation::print(std::ostream &os) const {
    os << robotPos_ << " ";
    if (seesTarget_) {
        os << "SEEN";
    } else {
        os << "UNSEEN";
    }
}

GridPosition TrackerObservation::getRobotPos() const {
    return robotPos_;
}

int TrackerObservation::getRobotYaw() const {
    return robotYaw_;
}

bool TrackerObservation::seesTarget() const {
    return seesTarget_;
}
}
/* namespace tracker */
