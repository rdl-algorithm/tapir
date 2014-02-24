#include "Nav2DAction.hpp"

#include <cstddef>                      // for size_t

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/geometry/State.hpp"             // for State

namespace nav2d {
Nav2DAction::ACTIONS = {

};

Nav2DAction::Nav2DAction(ActionType actionType, long rockNo) :
        actionType_(actionType),
        rockNo_(rockNo) {
}

Nav2DAction::Nav2DAction(long code) :
        actionType_(code <= 5 ? static_cast<ActionType>(code) : ActionType::CHECK),
        rockNo_(actionType_ == ActionType::CHECK ? code-5 : 0) {
}

std::unique_ptr<solver::Action> Nav2DAction::copy() const {
    return std::make_unique<Nav2DAction>(actionType_,rockNo_);
}

double Nav2DAction::distanceTo(solver::Action const &/*otherAction*/) const {
    return 0;
}

void Nav2DAction::print(std::ostream &os) const {
    if (actionType_ == ActionType::CHECK) {
        os << "CHECK-" << rockNo_;
        return;
    }
    switch (actionType_) {
    case ActionType::NORTH:
        os << "NORTH";
        break;
    case ActionType::EAST:
        os << "EAST";
        break;
    case ActionType::SOUTH:
        os << "SOUTH";
        break;
    case ActionType::WEST:
        os << "WEST";
        break;
    case ActionType::SAMPLE:
        os << "SAMPLE";
        break;
    default:
        os << "ERROR-" << static_cast<long>(actionType_);
        break;
    }
}

long Nav2DAction::getCode() const {
    long code = static_cast<long>(actionType_);
    if (actionType_ == ActionType::CHECK) {
        code += rockNo_;
    }
    return code;
}

ActionType Nav2DAction::getActionType() const {
    return actionType_;
}

long Nav2DAction::getRockNo() const {
    return rockNo_;
}
} /* namespace nav2d */
