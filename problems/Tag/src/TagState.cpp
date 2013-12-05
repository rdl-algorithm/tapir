#include "TagState.hpp"

#include <cstddef>                      // for size_t
#include <cmath>                        // for abs, pow

#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "GridPosition.hpp"             // for GridPosition
#include "State.hpp"                    // for State

#include <iostream> // temp debug

template<class T>
void hash_combine(std::size_t &seed, T const &v) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

void dispState(VectorState &s, std::ostream &os) {
    os << "ROBOT: " << decodeCoords(s.vals[0]) << " OPPONENT: "
       << decodeCoords(s.vals[1]);
    if (s.vals[2] == TAGGED) {
        os << " TAGGED!";
    }
}

TagState::TagState(GridPosition robotPos, GridPosition opponentPos, bool isTagged) :
    robotPos(robotPos),
    opponentPos(opponentPos),
    isTagged_(isTagged) {
}

TagState::TagState(TagState const &other) :
        State(),
        robotPos(other.robotPos),
        opponentPos(other.opponentPos),
        isTagged_(other.isTagged_) {
}

double TagState::distanceTo(State const &otherState) const {
    TagState const *otherTagState =
            static_cast<TagState const *>(&otherState);
    double distance = robotPos.distanceTo(otherTagState->robotPos);
    distance += opponentPos.distanceTo(otherTagState->opponentPos);
    distance += (isTagged_ == otherTagState->isTagged_) ? 1 : 0;
    return distance;
}

bool TagState::equals(State const &otherState) const {
    TagState const *otherTagState =
            static_cast<TagState const *>(&otherTagState);
    return (robotPos == otherTagState->robotPos &&
            opponentPos == otherTagState->opponentPos &&
            isTagged_ == otherTagState->isTagged_);
}

std::size_t TagState::hash() const {
    std::size_t hashValue = 0;
    hash_combine(hashValue, robotPos.i);
    hash_combine(hashValue, robotPos.j);
    hash_combine(hashValue, opponentPo.i);
    hash_combine(hashValue, opponentPos.j);
    hash_combine(hashValue, isTagged_);
    return hashValue;
}

void TagState::print(std::ostream &os) const {
    os << "ROBOT: " << robotPos << " OPPONENT: " << opponentPos;
    if (isTagged_) {
        os << " TAGGED!";
    }
}
