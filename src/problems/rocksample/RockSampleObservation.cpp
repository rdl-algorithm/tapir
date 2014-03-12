#include "RockSampleObservation.hpp"

#include <cstddef>                      // for size_t

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/abstract-problem/Observation.hpp"             // for Observation

namespace rocksample {

RockSampleObservation::RockSampleObservation() :
        isEmpty_(true),
        isGood_(false) {
}

RockSampleObservation::RockSampleObservation(bool _isGood) :
        isEmpty_(false),
        isGood_(_isGood) {
}

RockSampleObservation::RockSampleObservation(bool _isEmpty, bool _isGood) :
        isEmpty_(_isEmpty),
        isGood_(_isGood) {
}

RockSampleObservation::RockSampleObservation(long code) :
        isEmpty_(code == 0),
        isGood_(code == 1) {
}

std::unique_ptr<solver::Observation> RockSampleObservation::copy() const {
    return std::make_unique<RockSampleObservation>(isEmpty_,isGood_);
}

double RockSampleObservation::distanceTo(solver::Observation const &otherObs) const {
    RockSampleObservation const &other =
            static_cast<RockSampleObservation const &>(otherObs);
    return isGood_ == other.isGood_ ? 0 : 1;
}

bool RockSampleObservation::equals(solver::Observation const &otherObs) const {
    RockSampleObservation const &other =
        static_cast<RockSampleObservation const &>(otherObs);
    return isGood_ == other.isGood_;
}

std::size_t RockSampleObservation::hash() const {
    return isGood_ ? 1 : 0;
}

void RockSampleObservation::print(std::ostream &os) const {
    if (isEmpty_) {
        os << "NONE";
    } else if (isGood_) {
        os << "GOOD";
    } else {
        os << "BAD";
    }
}

long RockSampleObservation::getBinNumber() const {
    return isEmpty_ ? 0 : (isGood_ ? 1 : 2);
}

bool RockSampleObservation::isEmpty() const {
    return isEmpty_;
}

bool RockSampleObservation::isGood() const {
    return isGood_;
}
}
/* namespace rocksample */
