#include "TrackerState.hpp"

#include <cstddef>                      // for size_t

#include <functional>   // for hash
#include <ostream>                      // for operator<<, ostream, basic_ostream>
#include <vector>

#include "global.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/abstract-problem/State.hpp"             // for State

namespace tracker {
TrackerState::TrackerState(GridPosition robotPos, int robotYaw, 
        GridPosition targetPos, int targetYaw, bool seesTarget) :
    solver::Vector(),
    robotPos_(robotPos),
    robotYaw_(robotYaw),
    targetPos_(targetPos),
    targetYaw_(targetYaw),
    seesTarget_(seesTarget) {
}

TrackerState::TrackerState(TrackerState const &other) :
        TrackerState(other.robotPos_, other.robotYaw_,
            other.targetPos_, other.targetYaw_, other.seesTarget_) {
}

std::unique_ptr<solver::Point> TrackerState::copy() const {
    return std::make_unique<TrackerState>(robotPos_, robotYaw_, targetPos_, targetYaw_, seesTarget_);
}

double TrackerState::distanceTo(solver::State const &otherState) const {
    TrackerState const &otherTrackerState = static_cast<TrackerState const &>(otherState);
    double distance = robotPos_.manhattanDistanceTo(otherTrackerState.robotPos_);
    distance += targetPos_.manhattanDistanceTo(otherTrackerState.targetPos_);
    return distance;
}

bool TrackerState::equals(solver::State const &otherState) const {
    TrackerState const &otherTrackerState = static_cast<TrackerState const &>(otherState);
    return (robotPos_ == otherTrackerState.robotPos_
            && targetPos_ == otherTrackerState.targetPos_
            && seesTarget_ == otherTrackerState.seesTarget_
            && robotYaw_ == otherTrackerState.robotYaw_
            && targetYaw_ == otherTrackerState.targetYaw_);
}

std::size_t TrackerState::hash() const {
    std::size_t hashValue = 0;
    abt::hash_combine(hashValue, robotPos_.i);
    abt::hash_combine(hashValue, robotPos_.j);
    abt::hash_combine(hashValue, robotYaw_);
    abt::hash_combine(hashValue, targetPos_.i);
    abt::hash_combine(hashValue, targetPos_.j);
    abt::hash_combine(hashValue, targetYaw_);
    abt::hash_combine(hashValue, seesTarget_);
    return hashValue;
}

std::vector<double> TrackerState::asVector() const {
    std::vector<double> vec(7);
    vec[0] = robotPos_.i;
    vec[1] = robotPos_.j;
    vec[2] = robotYaw_;
    vec[3] = targetPos_.i;
    vec[4] = targetPos_.j;
    vec[5] = targetYaw_;
    vec[6] = seesTarget_ ? 1 : 0;
    return vec;
}

void TrackerState::print(std::ostream &os) const {
    os << "ROBOT: " << robotPos_ << " TARGET: " << targetPos_ << 
    "ROBOT YAW: " << robotYaw_ << " TARGET YAW: " << targetYaw_;
    if (seesTarget_) {
        os << " Sees target!";
    }
}


GridPosition TrackerState::getRobotPos() const {
    return robotPos_;
}

GridPosition TrackerState::getTargetPos() const {
    return targetPos_;
}

int TrackerState::getRobotYaw() const {
    return robotYaw_;
}

int TrackerState::getTargetYaw() const {
    return targetYaw_;
}

bool TrackerState::seesTarget() const {
    return seesTarget_;
}
} /* namespace tracker */
