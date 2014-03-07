#include "Nav2DState.hpp"

#include <cstddef>                      // for size_t
#include <cmath>                        // for round

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"
#include "problems/shared/geometry/Point2D.hpp"  // for Point2D
#include "solver/abstract-problem/State.hpp"             // for State
#include "problems/shared/geometry/utilities.hpp"

namespace nav2d {
Nav2DState::Nav2DState(double x, double y, double direction,
        double costPerUnitDistance, double costPerRevolution) :
                Nav2DState(geometry::Point2D(x, y), direction,
                        costPerUnitDistance, costPerRevolution) {
}

Nav2DState::Nav2DState(geometry::Point2D position, double direction,
            double costPerUnitDistance, double costPerRevolution) :
    position_(position),
    direction_(geometry::normalizeTurn(direction)),
    costPerUnitDistance_(costPerUnitDistance),
    costPerRevolution_(costPerRevolution) {
}

Nav2DState::~Nav2DState() {
}

Nav2DState::Nav2DState(Nav2DState const &other) :
        Nav2DState(other.position_, other.direction_,
                other.costPerUnitDistance_, other.costPerRevolution_) {
}

std::unique_ptr<solver::Point> Nav2DState::copy() const {
    return std::make_unique<Nav2DState>(*this);
}

double Nav2DState::distanceTo(solver::State const &otherState) const {
    Nav2DState const &other = static_cast<Nav2DState const &>(otherState);
    return costPerUnitDistance_ * std::sqrt(position_.distanceTo(
            other.position_)) + costPerRevolution_ * std::abs(
                    geometry::normalizeTurn(direction_ - other.direction_));
}

double Nav2DState::round(double value) const {
    return std::round(value * 1e6) / 1e6;
}

bool Nav2DState::equals(solver::State const &otherState) const {
    Nav2DState const &other =
        static_cast<Nav2DState const &>(otherState);
    return (round(position_.getX()) == round(other.position_.getX())
            && round(position_.getY()) == round(other.position_.getY())
            && round(direction_) == round(other.direction_));
}

std::size_t Nav2DState::hash() const {
    std::size_t hashValue = 0;
    abt::hash_combine(hashValue, round(position_.getX()));
    abt::hash_combine(hashValue, round(position_.getY()));
    abt::hash_combine(hashValue, round(direction_));
    return hashValue;
}

void Nav2DState::print(std::ostream &os) const {
    os << position_ << ":" << direction_;
}

std::vector<double> Nav2DState::asVector() const {
    return std::vector<double> {position_.x_, position_.y_, direction_};
}

geometry::Point2D Nav2DState::getPosition() const {
    return position_;
}

double Nav2DState::getX() const {
    return position_.x_;
}

double Nav2DState::getY() const {
    return position_.y_;
}

double Nav2DState::getDirection() const {
    return direction_;
}
} /* namespace nav2d */
