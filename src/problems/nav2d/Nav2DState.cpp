#include "Nav2DState.hpp"

#include <cstddef>                      // for size_t

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/geometry/State.hpp"             // for State

namespace nav2d {
Nav2DState::Nav2DState(double x, double y, double direction,
        double costPerUnitDistance, double costPerRevolution) :
                costPerUnitDistance_(costPerUnitDistance),
                costPerRevolution_(costPerRevolution),
                x_(x),
                y_(y),
                direction_(direction) {
}

Nav2DState::Nav2DState(Nav2DState const &other) :
        Nav2DState(other.x_, other.y_, other.direction_,
                other.costPerUnitDistance_, other.costPerRevolution_) {
}


std::unique_ptr<solver::Point> Nav2DState::copy() const {
    return std::make_unique<Nav2DState>(this);
}

double Nav2DState::distanceTo(solver::State const &otherState) const {
    Nav2DState const &other = static_cast<Nav2DState const &>(otherState);
    double distanceSquared = std::pow(x_ - other.x_, 2) + std::pow(
            y_ - other.y_, 2);
    return (std::sqrt(distanceSquared) * costPerUnitDistance_ +
            std::abs(normalizeAngle(direction_ - other.direction_))
            * costPerRevolution_);
}

bool Nav2DState::equals(solver::State const &otherState) const {
    Nav2DState const &other =
        static_cast<Nav2DState const &>(otherState);
    return x_ == other.x_ && y_ == other.y_ && direction_ == other.direction_;
}

std::size_t Nav2DState::hash() const {
    std::size_t hashValue = 0;
    abt::hash_combine(hashValue, x_);
    abt::hash_combine(hashValue, y_);
    abt::hash_combine(hashValue, direction_);
    return hashValue;
}

void Nav2DState::print(std::ostream &os) const {
    os << "(" << x_ << ", " << y_ << "):" << direction_;
}

std::vector<double> Nav2DState::asVector() const {
    return std::vector<double> {x_, y_, direction_};
}


double Nav2DState::getX() const {
    return x_;
}

double Nav2DState::getY() const {
    return y_;
}

double Nav2DState::getDirection() const {
    return direction_;
}
} /* namespace nav2d */
