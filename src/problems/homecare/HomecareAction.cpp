#include "HomecareAction.hpp"

#include <cstddef>                      // for size_t

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/abstract-problem/State.hpp"             // for State

namespace homecare {
HomecareAction::HomecareAction(ActionType actionType):
        actionType_(actionType) {
}

HomecareAction::HomecareAction(long code) :
        actionType_(static_cast<ActionType>(code)) {
}

std::unique_ptr<solver::Action> HomecareAction::copy() const {
    return std::make_unique<HomecareAction>(actionType_);
}

double HomecareAction::distanceTo(solver::Action const &/*otherAction*/) const {
    return 0;
}

void HomecareAction::print(std::ostream &os) const {
    switch (actionType_) {
    case ActionType::NORTH:
        os << "NORTH";
        break;
    case ActionType::NORTH_EAST:
        os << "NORTH_EAST";
        break;
    case ActionType::EAST:
        os << "EAST";
        break;
    case ActionType::SOUTH_EAST:
        os << "SOUTH_EAST";
        break;
    case ActionType::SOUTH:
        os << "SOUTH";
        break;
    case ActionType::SOUTH_WEST:
        os << "SOUTH_WEST";
        break;
    case ActionType::WEST:
        os << "WEST";
        break;
    case ActionType::NORTH_WEST:
        os << "NORTH_WEST";
        break;
    case ActionType::WAIT:
        os << "WAIT";
        break;
    default:
        os << "ERROR-" << static_cast<long>(actionType_);
        break;
    }
}

long HomecareAction::getBinNumber() const {
    return static_cast<long>(actionType_);
}
ActionType HomecareAction::getActionType() const {
    return actionType_;
}
} /* namespace homecare */
