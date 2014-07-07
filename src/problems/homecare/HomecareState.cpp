/** @file HomecareState.cpp
 *
 * Contains the implementation for the methods of HomecareState.
 */
#include "HomecareState.hpp"

#include <cstddef>                      // for size_t

#include <functional>   // for hash
#include <ostream>                      // for operator<<, ostream, basic_ostream>
#include <vector>

#include "global.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/abstract-problem/State.hpp"             // for State

namespace homecare {
HomecareState::HomecareState(GridPosition robotPos, GridPosition targetPos,
        bool call) :
    solver::Vector(),
    robotPos_(robotPos),
    targetPos_(targetPos),
    call_(call) {
}

HomecareState::HomecareState(HomecareState const &other) :
        HomecareState(other.robotPos_, other.targetPos_, other.call_) {
}

std::unique_ptr<solver::Point> HomecareState::copy() const {
    return std::make_unique<HomecareState>(robotPos_, targetPos_, call_);
}

double HomecareState::distanceTo(solver::State const &otherState) const {
    HomecareState const &otherHomecareState = static_cast<HomecareState const &>(otherState);
    double distance = robotPos_.manhattanDistanceTo(otherHomecareState.robotPos_);
    distance += targetPos_.manhattanDistanceTo(otherHomecareState.targetPos_);
    distance += (call_ == otherHomecareState.call_) ? 0 : 1;
    return distance;
}

bool HomecareState::equals(solver::State const &otherState) const {
    HomecareState const &otherHomecareState = static_cast<HomecareState const &>(otherState);
    return (robotPos_ == otherHomecareState.robotPos_
            && targetPos_ == otherHomecareState.targetPos_
            && call_ == otherHomecareState.call_);
}

std::size_t HomecareState::hash() const {
    std::size_t hashValue = 0;
    tapir::hash_combine(hashValue, robotPos_.i);
    tapir::hash_combine(hashValue, robotPos_.j);
    tapir::hash_combine(hashValue, targetPos_.i);
    tapir::hash_combine(hashValue, targetPos_.j);
    tapir::hash_combine(hashValue, call_);
    return hashValue;
}

std::vector<double> HomecareState::asVector() const {
    std::vector<double> vec(5);
    vec[0] = robotPos_.i;
    vec[1] = robotPos_.j;
    vec[2] = targetPos_.i;
    vec[3] = targetPos_.j;
    vec[4] = call_ ? 1 : 0;
    return vec;
}

void HomecareState::print(std::ostream &os) const {
    os << "ROBOT: " << robotPos_ << " TARGET: " << targetPos_;
    if (call_) {
        os << " CALLING!";
    } else {
        os << " NOT CALLING";
    }
}


GridPosition HomecareState::getRobotPos() const {
    return robotPos_;
}

GridPosition HomecareState::getTargetPos() const {
    return targetPos_;
}

bool HomecareState::getCall() const {
    return call_;
}
} /* namespace homecare */
