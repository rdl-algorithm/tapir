#include "TrackerAction.hpp"

#include <cstddef>                      // for size_t

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/abstract-problem/State.hpp"             // for State

namespace tracker {
TrackerAction::TrackerAction(ActionType actionType):
        actionType_(actionType) {
}

TrackerAction::TrackerAction(long code) :
        actionType_(static_cast<ActionType>(code)) {
}

std::unique_ptr<solver::Action> TrackerAction::copy() const {
    return std::make_unique<TrackerAction>(actionType_);
}

double TrackerAction::distanceTo(solver::Action const &/*otherAction*/) const {
    return 0;
}

void TrackerAction::print(std::ostream &os) const {
    switch (actionType_) {
    case ActionType::FORWARD:
        os << "FORWARD";
        break;
    case ActionType::TURN_RIGHT:
        os << "TURN_RIGHT";
        break;
    case ActionType::TURN_LEFT:
        os << "TURN_LEFT";
        break;
    case ActionType::REVERSE:
        os << "REVERSE";
        break;
    case ActionType::WAIT:
        os << "WAIT";
        break;
    default:
        os << "ERROR-" << static_cast<long>(actionType_);
        break;
    }
}

long TrackerAction::getBinNumber() const {
    return static_cast<long>(actionType_);
}
ActionType TrackerAction::getActionType() const {
    return actionType_;
}
} /* namespace tracker */
