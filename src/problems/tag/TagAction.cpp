#include "TagAction.hpp"

#include <cstddef>                      // for size_t

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/topology/State.hpp"             // for State

namespace tag {
TagAction::TagAction(ActionType actionType):
        actionType_(actionType) {
}

TagAction::TagAction(long code) :
        actionType_(static_cast<ActionType>(code)) {
}

std::unique_ptr<solver::Action> TagAction::copy() const {
    return std::make_unique<TagAction>(actionType_);
}

double TagAction::distanceTo(solver::Action const &/*otherAction*/) const {
    return 0;
}

void TagAction::print(std::ostream &os) const {
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
    case ActionType::TAG:
        os << "TAG";
        break;
    default:
        os << "ERROR-" << static_cast<long>(actionType_);
        break;
    }
}

long TagAction::getCode() const {
    return static_cast<long>(actionType_);
}

ActionType TagAction::getActionType() const {
    return actionType_;
}
} /* namespace tag */
