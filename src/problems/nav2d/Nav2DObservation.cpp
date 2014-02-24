#include "Nav2DObservation.hpp"

#include <cstddef>                      // for size_t

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/geometry/Observation.hpp"             // for Observation

namespace nav2d {

Nav2DObservation::Nav2DObservation() :
        isEmpty_(true),
        isGood_(false) {
}

Nav2DObservation::Nav2DObservation(bool _isGood) :
        isEmpty_(false),
        isGood_(_isGood) {
}

Nav2DObservation::Nav2DObservation(bool _isEmpty, bool _isGood) :
        isEmpty_(_isEmpty),
        isGood_(_isGood) {
}

Nav2DObservation::Nav2DObservation(long code) :
        isEmpty_(code == 0),
        isGood_(code == 1) {
}

std::unique_ptr<solver::Observation> Nav2DObservation::copy() const {
    return std::make_unique<Nav2DObservation>(isEmpty_,isGood_);
}

double Nav2DObservation::distanceTo(solver::Observation const &otherObs) const {
    Nav2DObservation const &other =
            static_cast<Nav2DObservation const &>(otherObs);
    return isGood_ == other.isGood_ ? 0 : 1;
}

bool Nav2DObservation::equals(solver::Observation const &otherObs) const {
    Nav2DObservation const &other =
        static_cast<Nav2DObservation const &>(otherObs);
    return isGood_ == other.isGood_;
}

std::size_t Nav2DObservation::hash() const {
    return isGood_ ? 1 : 0;
}

void Nav2DObservation::print(std::ostream &os) const {
    if (isEmpty_) {
        os << "NONE";
    } else if (isGood_) {
        os << "GOOD";
    } else {
        os << "BAD";
    }
}

long Nav2DObservation::getCode() const {
    return isEmpty_ ? 0 : (isGood_ ? 1 : 2);
}

bool Nav2DObservation::isEmpty() const {
    return isEmpty_;
}

bool Nav2DObservation::isGood() const {
    return isGood_;
}
}
/* namespace nav2d */
