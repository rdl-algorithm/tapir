/** @file RockSampleState.cpp
 *
 * Contains the implementation for the methods of RockSampleState.
 */
#include "RockSampleState.hpp"

#include <cstddef>                      // for size_t

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/abstract-problem/State.hpp"             // for State

namespace rocksample {
RockSampleState::RockSampleState(GridPosition position,
        std::vector<bool> rockStates) :
    solver::Vector(),
    position_(position),
    rockStates_(rockStates) {
}

std::unique_ptr<solver::Point> RockSampleState::copy() const {
    return std::make_unique<RockSampleState>(position_, rockStates_);
}

double RockSampleState::distanceTo(solver::State const &otherState) const {
    RockSampleState const &otherRockSampleState =
        static_cast<RockSampleState const &>(otherState);
    double distance = position_.manhattanDistanceTo(
                otherRockSampleState.position_) / 10.0;
    typedef std::vector<bool>::const_iterator BoolIt;
    BoolIt it1 = rockStates_.cbegin();
    BoolIt it2 = otherRockSampleState.rockStates_.cbegin();
    for (; it1 != rockStates_.cend(); it1++, it2++) {
        if (*it1 != *it2) {
            distance += 1;
        }
    }
    return distance;
}

bool RockSampleState::equals(solver::State const &otherState) const {
    RockSampleState const &otherRockSampleState =
        static_cast<RockSampleState const &>(otherState);
    return (position_ == otherRockSampleState.position_
            && rockStates_ == otherRockSampleState.rockStates_);
}

std::size_t RockSampleState::hash() const {
    std::size_t hashValue = 0;
    tapir::hash_combine(hashValue, position_.i);
    tapir::hash_combine(hashValue, position_.j);
    tapir::hash_combine(hashValue, rockStates_);
    return hashValue;
}

void RockSampleState::print(std::ostream &os) const {
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


std::vector<double> RockSampleState::asVector() const {
    std::vector<double> vec(2 + rockStates_.size());
    vec[0] = position_.i;
    vec[1] = position_.j;
    for (std::size_t i = 0; i < rockStates_.size(); i++) {
        vec[i + 2] = rockStates_[i] ? 1 : 0;
    }
    return vec;
}

GridPosition RockSampleState::getPosition() const {
     return position_;
}

 std::vector<bool> RockSampleState::getRockStates() const {
     return rockStates_;
 }
} /* namespace rocksample */
