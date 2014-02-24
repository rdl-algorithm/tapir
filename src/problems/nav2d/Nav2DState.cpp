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
Nav2DState::Nav2DState(GridPosition position,
        std::vector<bool> rockStates) :
    solver::Vector(),
    position_(position),
    rockStates_(rockStates) {
}

Nav2DState::Nav2DState(Nav2DState const &other) :
    Nav2DState(other.position_, other.rockStates_) {
}

std::unique_ptr<solver::Point> Nav2DState::copy() const {
    return std::make_unique<Nav2DState>(position_, rockStates_);
}

double Nav2DState::distanceTo(solver::State const &otherState) const {
    Nav2DState const &otherNav2DState =
        static_cast<Nav2DState const &>(otherState);
    double distance = position_.manhattanDistanceTo(
                otherNav2DState.position_) / 10.0;
    typedef std::vector<bool>::const_iterator BoolIt;
    BoolIt it1 = rockStates_.cbegin();
    BoolIt it2 = otherNav2DState.rockStates_.cbegin();
    for (; it1 != rockStates_.cend(); it1++, it2++) {
        if (*it1 != *it2) {
            distance += 1;
        }
    }
    return distance;
}

bool Nav2DState::equals(solver::State const &otherState) const {
    Nav2DState const &otherNav2DState =
        static_cast<Nav2DState const &>(otherState);
    return (position_ == otherNav2DState.position_
            && rockStates_ == otherNav2DState.rockStates_);
}

std::size_t Nav2DState::hash() const {
    std::size_t hashValue = 0;
    abt::hash_combine(hashValue, position_.i);
    abt::hash_combine(hashValue, position_.j);
    abt::hash_combine(hashValue, rockStates_);
    return hashValue;
}

void Nav2DState::print(std::ostream &os) const {
    os << position_ << " GOOD: {";
    std::vector<int> goodRocks;
    std::vector<int> badRocks;
    for (std::size_t i = 0; i < rockStates_.size(); i++) {
        if (rockStates_[i]) {
            goodRocks.push_back(i);
        } else {
            badRocks.push_back(i);
        }
    }
    std::copy(goodRocks.begin(), goodRocks.end(),
            std::ostream_iterator<double>(os, " "));
    os << "}; BAD: {";
    std::copy(badRocks.begin(), badRocks.end(),
            std::ostream_iterator<double>(os, " "));
    os << "}";
}


std::vector<double> Nav2DState::asVector() const {
    std::vector<double> vec(2 + rockStates_.size());
    vec[0] = position_.i;
    vec[1] = position_.j;
    for (std::size_t i = 0; i < rockStates_.size(); i++) {
        vec[i + 2] = rockStates_[i] ? 1 : 0;
    }
    return vec;
}

GridPosition Nav2DState::getPosition() const {
     return position_;
}

 std::vector<bool> Nav2DState::getRockStates() const {
     return rockStates_;
 }
} /* namespace nav2d */
