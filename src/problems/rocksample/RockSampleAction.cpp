#include "RockSampleAction.hpp"

#include <cstddef>                      // for size_t

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/State.hpp"             // for State

namespace rocksample {
RockSampleAction::RockSampleAction(Code code) :
        code_(code),
        rockNo_(0) {
}

RockSampleAction::RockSampleAction(Code code, long rockNo) :
        code_(code),
        rockNo_(rockNo) {
}

std::unique_ptr<solver::Action> RockSampleAction::copy() const {
    return std::make_unique<RockSampleAction>(code_,rockNo_);
}

double RockSampleAction::distanceTo(solver::Action const &/*otherAction*/) const {
    return 0;
}

bool RockSampleAction::equals(solver::Action const &otherAction) const {
    RockSampleAction const &other =
        static_cast<RockSampleAction const &>(otherAction);
    return (code_ == other.code_
            && rockNo_ == other.rockNo_);
}

std::size_t RockSampleAction::hash() const {
    return static_cast<long>(code_) + rockNo_;
}

void RockSampleAction::print(std::ostream &os) const {
    if (code_ == Code::CHECK) {
        os << "CHECK-" << rockNo_;
        return;
    }
    switch (code_) {
    case Code::NORTH:
        os << "NORTH";
        break;
    case Code::EAST:
        os << "EAST";
        break;
    case Code::SOUTH:
        os << "SOUTH";
        break;
    case Code::WEST:
        os << "WEST";
        break;
    case Code::SAMPLE:
        os << "SAMPLE";
        break;
    default:
        os << "ERROR-" << static_cast<long>(code_);
        break;
    }
}

RockSampleAction::Code RockSampleAction::getCode() const {
    return code_;
}

long RockSampleAction::getRockNo() const {
    return rockNo_;
}
} /* namespace rocksample */
