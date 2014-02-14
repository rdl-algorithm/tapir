#include "Nav2DTextSerializer.hpp"

#include <iostream>                     // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, basic_istream<>::__istream_type

#include "global.hpp"                     // for make_unique
#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/State.hpp"             // for State
#include "solver/TextSerializer.hpp"    // for TextSerializer

#include "Nav2DState.hpp"                 // for Nav2DState

namespace solver {
class Solver;
} /* namespace solver */

namespace nav2d {
Nav2DTextSerializer::Nav2DTextSerializer() :
    solver::TextSerializer() {
}

Nav2DTextSerializer::Nav2DTextSerializer(solver::Solver *solver) :
    solver::TextSerializer(solver) {
}

void Nav2DTextSerializer::saveState(solver::State const &state, std::ostream &os) {
    Nav2DState const &nav2dState = static_cast<Nav2DState const &>(state);
    os << nav2dState.x_ << " " << nav2dState.y_ << " " << nav2dState.numTurns_;
}

std::unique_ptr<solver::State> Nav2DTextSerializer::loadState(std::istream &is) {
    long i, j;
    is >> i >> j;
    GridPosition robotPos(i, j);
    is >> i >> j;
    GridPosition opponentPos(i, j);
    bool isNav2Dged;
    is >> isNav2Dged;
    return std::make_unique<Nav2DState>(robotPos, opponentPos, isNav2Dged);
}
} /* namespace nav2d */
