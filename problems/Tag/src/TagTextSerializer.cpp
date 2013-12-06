#include "TagTextSerializer.hpp"

#include <iostream>                     // for cerr, endl
#include <string>                       // for string

#include "defs.hpp"                     // for make_unique
#include "GridPosition.hpp"             // for GridPosition
#include "TagState.hpp"                 // for TagState
#include "TextSerializer.hpp"           // for TextSerializer

TagTextSerializer::TagTextSerializer() :
            TextSerializer() {
}

TagTextSerializer::TagTextSerializer(Solver *solver) :
            TextSerializer(solver) {
}


void TagTextSerializer::saveState(State &state, std::ostream &os) {
    TagState *tagState = static_cast<TagState *>(&state);
    os << tagState->robotPos.i << " " << tagState->robotPos.j << " " <<
            tagState->opponentPos.i << " " << tagState->opponentPos.j << " "
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

