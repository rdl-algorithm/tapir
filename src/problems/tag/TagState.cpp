#include "TagState.hpp"

#include <cstddef>                      // for size_t

#include <functional>   // for hash
#include <ostream>                      // for operator<<, ostream, basic_ostream>
#include <vector>

#include "global.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/abstract-problem/State.hpp"             // for State

namespace tag {
TagState::TagState(GridPosition robotPos, GridPosition opponentPos,
        bool _isTagged) :
    solver::Vector(),
    robotPos_(robotPos),
    opponentPos_(opponentPos),
    isTagged_(_isTagged) {
}

TagState::TagState(TagState const &other) :
        TagState(other.robotPos_, other.opponentPos_, other.isTagged_) {
}

std::unique_ptr<solver::Point> TagState::copy() const {
    return std::make_unique<TagState>(robotPos_, opponentPos_, isTagged_);
}

double TagState::distanceTo(solver::State const &otherState) const {
    TagState const &otherTagState = static_cast<TagState const &>(otherState);
    double distance = robotPos_.manhattanDistanceTo(otherTagState.robotPos_);
    distance += opponentPos_.manhattanDistanceTo(otherTagState.opponentPos_);
    distance += (isTagged_ == otherTagState.isTagged_) ? 0 : 1;
    return distance;
}

bool TagState::equals(solver::State const &otherState) const {
    TagState const &otherTagState = static_cast<TagState const &>(otherState);
    return (robotPos_ == otherTagState.robotPos_
            && opponentPos_ == otherTagState.opponentPos_
            && isTagged_ == otherTagState.isTagged_);
}

std::size_t TagState::hash() const {
    std::size_t hashValue = 0;
    abt::hash_combine(hashValue, robotPos_.i);
    abt::hash_combine(hashValue, robotPos_.j);
    abt::hash_combine(hashValue, opponentPos_.i);
    abt::hash_combine(hashValue, opponentPos_.j);
    abt::hash_combine(hashValue, isTagged_);
    return hashValue;
}

std::vector<double> TagState::asVector() const {
    std::vector<double> vec(5);
    vec[0] = robotPos_.i;
    vec[1] = robotPos_.j;
    vec[2] = opponentPos_.i;
    vec[3] = opponentPos_.j;
    vec[4] = isTagged_ ? 1 : 0;
    return vec;
}

void TagState::print(std::ostream &os) const {
    os << "ROBOT: " << robotPos_ << " OPPONENT: " << opponentPos_;
    if (isTagged_) {
        os << " TAGGED!";
    }
}


GridPosition TagState::getRobotPosition() const {
    return robotPos_;
}

GridPosition TagState::getOpponentPosition() const {
    return opponentPos_;
}

bool TagState::isTagged() const {
    return isTagged_;
}
} /* namespace tag */
