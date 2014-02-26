#include "Nav2DAction.hpp"

#include <cstddef>                      // for size_t

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/geometry/State.hpp"             // for State

namespace nav2d {
Nav2DAction::Nav2DAction(double speed, double rotationalSpeed) :
        speed_(speed),
        rotationalSpeed_(rotationalSpeed) {
}

Nav2DAction::Nav2DAction(Nav2DAction const &other) :
        Nav2DAction(other.speed_, other.rotationalSpeed_) {
}

std::unique_ptr<solver::Action> Nav2DAction::copy() const {
    return std::make_unique<Nav2DAction>(*this);
}

double Nav2DAction::distanceTo(solver::Action const &/*otherAction*/) const {
    return 0;
}

bool Nav2DAction::equals(solver::Action const &otherAction) const {
    Nav2DAction const &other = static_cast<Nav2DAction const &>(otherAction);
    return (speed_ == other.speed_
            && rotationalSpeed_ == other.rotationalSpeed_);
}

std::size_t Nav2DAction::hash() const {
    std::size_t hashValue = 0;
    abt::hash_combine(hashValue, speed_);
    abt::hash_combine(hashValue, rotationalSpeed_);
    return hashValue;
}

void Nav2DAction::print(std::ostream &os) const {
    os << speed_ << " / " << rotationalSpeed_;
}

double Nav2DAction::getSpeed() const {
    return speed_;
}

double Nav2DAction::getRotationalSpeed() const{
    return rotationalSpeed_;
}
} /* namespace nav2d */
