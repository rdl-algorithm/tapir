#include "Nav2DState.hpp"

#include <cstddef>                      // for size_t
#include <cmath>                        // for pow

#include <functional>   // for hash
#include <ostream>                      // for operator<<, ostream, basic_ostream>
#include <vector>

#include "defs.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/State.hpp"             // for State

namespace nav2d {
constexpr double normalizeTurns(double numTurns) {
    double value = std::fmod(numTurns, 1);
    return value <= -0.5 ? value + 1 : (value > 0.5 ? value-1 : value);
}

constexpr double DISTANCE_PER_TURN = 1;

Nav2DState::Nav2DState(double x, double y, double numTurns) :
    solver::Vector(),
    x_(x),
    y_(y),
    numTurns_(normalizeTurns(numTurns)) {
}

Nav2DState::Nav2DState(Nav2DState const &other) :
        Nav2DState(other.x_, other.y_, other.numTurns_) {
}

std::unique_ptr<solver::Point> Nav2DState::copy() const {
    return std::make_unique<Nav2DState>(this);
}

double Nav2DState::distanceTo(solver::State const &otherState) const {
    Nav2DState const &other = static_cast<Nav2DState const &>(other);
    double squaredDistance = std::pow(x_ - other.x_, 2);
    squaredDistance += std::pow(y_ - other.y_, 2);
    double numTurns = std::abs(normalizeTurns(numTurns_ - other.numTurns_));
    return std::sqrt(squaredDistance) + DISTANCE_PER_TURN * numTurns;
}

bool Nav2DState::equals(solver::State const &otherState) const {
    Nav2DState const &other = static_cast<Nav2DState const &>(other);
    return (x_ == other.x_ && y_ == other.y_ && numTurns_ == other.numTurns_);
}

std::size_t Nav2DState::hash() const {
    std::size_t hashValue = 0;
    hash_combine(hashValue, x_);
    hash_combine(hashValue, y_);
    hash_combine(hashValue, numTurns_);
    return hashValue;
}

std::vector<double> Nav2DState::asVector() const {
    return std::vector<double> {x_, y_, numTurns_};
}

void Nav2DState::print(std::ostream &os) const {
    os << "(" << x_ << ", " << y_ << ") < " << numTurns_ * 360 << " deg";
}
} /* namespace nav2d */
