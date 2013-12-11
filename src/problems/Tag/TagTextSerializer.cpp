#include "TagTextSerializer.hpp"

#include <iostream>                     // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, basic_istream<>::__istream_type

#include "defs.hpp"                     // for make_unique
#include "problems/GridPosition.hpp"    // for GridPosition
#include "solver/State.hpp"             // for State
#include "solver/TextSerializer.hpp"    // for TextSerializer

#include "TagState.hpp"                 // for TagState

class Solver;

TagTextSerializer::TagTextSerializer() :
    TextSerializer() {
}

TagTextSerializer::TagTextSerializer(Solver *solver) :
    TextSerializer(solver) {
}


void TagTextSerializer::saveState(State &state, std::ostream &os) {
    TagState *tagState = static_cast<TagState *>(&state);
    os << tagState->robotPos.i << " " << tagState->robotPos.j << " "
       <<tagState->opponentPos.i << " " << tagState->opponentPos.j << " "
       << tagState->isTagged_;
}

std::unique_ptr<State> TagTextSerializer::loadState(std::istream &is) {
    long i, j;
    is >> i >> j;
    GridPosition robotPos(i, j);
    is >> i >> j;
    GridPosition opponentPos(i, j);
    bool isTagged;
    is >> isTagged;
    return std::make_unique<TagState>(robotPos, opponentPos, isTagged);
}

