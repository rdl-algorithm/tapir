#include "RockSampleState.hpp"

#include <cstddef>                      // for size_t

#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "GridPosition.hpp"             // for GridPosition
#include "State.hpp"                    // for State

template<class T>
void hash_combine(std::size_t &seed, T const &v) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

RockSampleState::RockSampleState(GridPosition position,
        std::vector<bool> rockStates) :
            State(),
            position(position),
            rockStates(rockStates) {
}

RockSampleState::RockSampleState(RockSampleState const &other) :
            State(),
            position(other.position),
            rockStates(other.rockStates) {
}

double RockSampleState::distanceTo(State const &otherState) const {
    RockSampleState const *otherRockSampleState =
            static_cast<RockSampleState const *>(&otherState);
    double distance = position.distanceTo(otherRockSampleState->position);
    distance /= 10.0;
    typedef std::vector<bool>::const_iterator BoolIt;
    BoolIt it1 = rockStates.cbegin();
    BoolIt it2 = otherRockSampleState->rockStates.cbegin();
    for (; it1 != rockStates.cend(); it1++, it2++) {
        if (*it1 != *it2) {
            distance += 1;
        }
    }
    return distance;
}

bool RockSampleState::equals(State const &otherState) const {
    RockSampleState const *otherRockSampleState =
            static_cast<RockSampleState const *>(&otherState);
    return (position == otherRockSampleState->position
            && rockStates == otherRockSampleState->rockStates);
}

std::size_t RockSampleState::hash() const {
    std::size_t hashValue = 0;
    hash_combine(hashValue, position.i);
    hash_combine(hashValue, position.j);
    hash_combine(hashValue, rockStates);
    return hashValue;
}

void RockSampleState::print(std::ostream &os) const {
    os << position << " GOOD: {";
    std::vector<int> goodRocks;
    std::vector<int> badRocks;
    for (std::size_t i = 0; i < rockStates.size(); i++) {
        if (rockStates[i]) {
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
