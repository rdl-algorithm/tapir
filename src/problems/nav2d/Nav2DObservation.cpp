#include "Nav2DObservation.hpp"

#include <cstddef>                      // for size_t

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <limits>
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/geometry/Observation.hpp"             // for Observation

namespace nav2d {

Nav2DObservation::Nav2DObservation() :
        state_(nullptr) {
}

Nav2DObservation::Nav2DObservation(Nav2DState const &state) :
        state_(state.copy()) {
}

Nav2DObservation::Nav2DObservation(double x, double y, double direction) :
        state_(std::make_unique<Nav2DState>(x, y, direction)) {
}


std::unique_ptr<solver::Observation> Nav2DObservation::copy() const {
    if (state_ == nullptr) {
        return std::make_unique<Nav2DObservation>();
    } else {
        return std::make_unique<Nav2DObservation>(state_);
    }
}

double Nav2DObservation::distanceTo(
        solver::Observation const &otherObs) const {
    Nav2DObservation const &other =
            static_cast<Nav2DObservation const &>(otherObs);
    if (state_ == nullptr || other.state_ == nullptr) {
        return std::numeric_limits<double>::infinity();
    }
    return state_->distanceTo(*other.state_);
}

bool Nav2DObservation::equals(solver::Observation const &otherObs) const {
    Nav2DObservation const &other =
        static_cast<Nav2DObservation const &>(otherObs);

    if (state_ == nullptr && other.state_ == nullptr) {
        return true;
    } else if (state_ == nullptr || other.state_ == nullptr) {
        return false;
    }
    return *state_ == *other.state_;
}

std::size_t Nav2DObservation::hash() const {
    if (state_ == nullptr) {
        return 0;
    }
    return state_->hash();
}

void Nav2DObservation::print(std::ostream &os) const {
    if (state_ == nullptr) {
        os << "NONE";
    } else {
        os << *state_;
    }
}


bool Nav2DObservation::isEmpty() const {
    return state_ == nullptr;
}

Nav2DState const *Nav2DObservation::getState() const {
    return state_->copy();
}

double Nav2DObservation::getX() const {
    return state_->x_;
}

double Nav2DObservation::getY() const {
    return state_->y_;
}

double Nav2DObservation::getDirection() const {
    return state_->direction_;
}
}
/* namespace nav2d */
