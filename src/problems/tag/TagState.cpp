#include "TagState.hpp"

#include <cstddef>                      // for size_t

#include <ostream>                      // for operator<<, ostream, basic_ostream
#include <unordered_map>                // for hash

#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/State.hpp"             // for State

namespace tag {
template<class T>
void hash_combine(std::size_t &seed, T const &v) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
TagState::TagState(GridPosition robotPos, GridPosition opponentPos,
        bool isTagged) :
    robotPos_(robotPos),
    opponentPos_(opponentPos),
    isTagged_(isTagged) {
}
#pragma GCC diagnostic pop

TagState::TagState(TagState const &other) :
    solver::State(),
    robotPos_(other.robotPos_),
    opponentPos_(other.opponentPos_),
    isTagged_(other.isTagged_) {
}

double TagState::distanceTo(State const &otherState) const {
    TagState const *otherTagState =
        static_cast<TagState const *>(&otherState);
    double distance = robotPos_.manhattanDistanceTo(otherTagState->robotPos_);
    distance += opponentPos_.manhattanDistanceTo(otherTagState->opponentPos_);
    distance += (isTagged_ == otherTagState->isTagged_) ? 0 : 1;
    return distance;
}

bool TagState::equals(State const &otherState) const {
    TagState const *otherTagState =
        static_cast<TagState const *>(&otherState);
    return (robotPos_ == otherTagState->robotPos_
            && opponentPos_ == otherTagState->opponentPos_
            && isTagged_ == otherTagState->isTagged_);
}

std::size_t TagState::hash() const {
    std::size_t hashValue = 0;
    hash_combine(hashValue, robotPos_.i);
    hash_combine(hashValue, robotPos_.j);
    hash_combine(hashValue, opponentPos_.i);
    hash_combine(hashValue, opponentPos_.j);
    hash_combine(hashValue, isTagged_);
    return hashValue;
}

void TagState::print(std::ostream &os) const {
    os << "ROBOT: " << robotPos_ << " OPPONENT: " << opponentPos_;
    if (isTagged_) {
        os << " TAGGED!";
    }
}
} /* namespace tag */
