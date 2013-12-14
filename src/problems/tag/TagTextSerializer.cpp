#include "TagTextSerializer.hpp"

#include <iostream>                     // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, basic_istream<>::__istream_type

#include "defs.hpp"                     // for make_unique
#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/State.hpp"             // for State
#include "solver/TextSerializer.hpp"    // for TextSerializer

#include "TagState.hpp"                 // for TagState

namespace solver {
class Solver;
} /* namespace solver */

namespace tag {
TagTextSerializer::TagTextSerializer() :
    solver::TextSerializer() {
}

TagTextSerializer::TagTextSerializer(solver::Solver *solver) :
    solver::TextSerializer(solver) {
}


void TagTextSerializer::saveState(solver::State &state, std::ostream &os) {
    TagState *tagState = static_cast<TagState *>(&state);
    os << tagState->robotPos_.i << " " << tagState->robotPos_.j << " "
       <<tagState->opponentPos_.i << " " << tagState->opponentPos_.j << " "
       << tagState->isTagged_;
}

std::unique_ptr<solver::State> TagTextSerializer::loadState(std::istream &is) {
    long i, j;
    is >> i >> j;
    GridPosition robotPos(i, j);
    is >> i >> j;
    GridPosition opponentPos(i, j);
    bool isTagged;
    is >> isTagged;
    return std::make_unique<TagState>(robotPos, opponentPos, isTagged);
}
} /* namespace tag */
