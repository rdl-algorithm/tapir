#include "RockSampleTextSerializer.hpp"

#include <iostream>                     // for cerr, endl
#include <string>                       // for string

#include "defs.hpp"                     // for make_unique
#include "GridPosition.hpp"             // for GridPosition
#include "RockSampleState.hpp"          // for RockSampleState
#include "TextSerializer.hpp"           // for TextSerializer

RockSampleTextSerializer::RockSampleTextSerializer() :
            TextSerializer() {
}

RockSampleTextSerializer::RockSampleTextSerializer(Solver *solver) :
            TextSerializer(solver) {
}


void RockSampleTextSerializer::saveState(State &state, std::ostream &os) {
    RockSampleState *rockSampleState =
        static_cast<RockSampleState *>(&state);
    os << rockSampleState->position.i << " " << rockSampleState->position.j << " ";
    for (bool isGood : rockSampleState->getRockStates()) {
        if (isGood) {
            os << 'G';
        } else {
            os << 'B';
        }
    }
}

std::unique_ptr<State> RockSampleTextSerializer::loadState(std::istream &is) {
    long i, j;
    std::string rockString;
    std::vector<bool> rockStates;
    is >> i >> j >> rockString;
    for (char c : rockString) {
        if (c == 'G') {
            rockStates.push_back(true);
        } else if (c == 'B') {
            rockStates.push_back(false);
        } else {
            std::cerr << "Error; invalid rock state: " << c << std::endl;
        }
    }
    return std::make_unique<RockSampleState>(GridPosition(i, j), rockStates);
}

