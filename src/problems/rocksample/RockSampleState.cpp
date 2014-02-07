#include "RockSampleState.hpp"

#include <cstddef>                      // for size_t

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "defs.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/State.hpp"             // for State

namespace rocksample {
template<class T>
void hash_combine(std::size_t &seed, T const &v) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

RockSampleState::RockSampleState(GridPosition position,
        std::vector<bool> rockStates) :
    solver::Vector(),
    position_(position),
    rockStates_(rockStates) {
}

RockSampleState::RockSampleState(RockSampleState const &other) :
    RockSampleState(other.position_, other.rockStates_) {
}

std::unique_ptr<solver::Point> RockSampleState::copy() const {
    return std::make_unique<RockSampleState>(position_, rockStates_);
}

double RockSampleState::distanceTo(solver::State const &otherState) const {
    RockSampleState const *otherRockSampleState =
        static_cast<RockSampleState const *>(&otherState);
    double distance = position_.manhattanDistanceTo(
                otherRockSampleState->position_) / 10.0;
    typedef std::vector<bool>::const_iterator BoolIt;
    BoolIt it1 = rockStates_.cbegin();
    BoolIt it2 = otherRockSampleState->rockStates_.cbegin();
    for (; it1 != rockStates_.cend(); it1++, it2++) {
        if (*it1 != *it2) {
            distance += 1;
        }
    }
    return distance;
}

bool RockSampleState::equals(solver::State const &otherState) const {
    RockSampleState const *otherRockSampleState =
        static_cast<RockSampleState const *>(&otherState);
    return (position_ == otherRockSampleState->position_
            && rockStates_ == otherRockSampleState->rockStates_);
}

std::size_t RockSampleState::hash() const {
    std::size_t hashValue = 0;
    hash_combine(hashValue, position_.i);
    hash_combine(hashValue, position_.j);
    hash_combine(hashValue, rockStates_);
    return hashValue;
}


std::vector<double> RockSampleState::asVector() const {
    std::vector<double> vec(2 + rockStates_.size());
    vec[0] = position_.i;
    vec[1] = position_.j;
    for (unsigned long i = 0; i < rockStates_.size(); i++) {
        vec[i + 2] = rockStates_[i] ? 1 : 0;
    }
    return vec;
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
} /* namespace rocksample */
