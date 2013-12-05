#include "RockSampleTextSerializer.hpp"

#include <iostream>                     // for cerr, endl
#include <string>                       // for string

#include "RockSampleState.hpp"          // for RockSampleState
#include "TextSerializer.hpp"           // for TextSerializer

RockSampleTextSerializer::RockSampleTextSerializer() :
            TextSerializer() {
}

RockSampleTextSerializer::RockSampleTextSerializer(Solver *solver) :
            TextSerializer(solver) {
}


void RockSampleTextSerializer::save(State &state, std::ostream &os) {
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

void RockSampleTextSerializer::load(State &state, std::istream &is) {
    RockSampleState *rockSampleState =
        static_cast<RockSampleState *>(&state);
    is >> rockSampleState->position.i;
    is >> rockSampleState->position.j;
    std::string rocks;
    is >> rocks;
    rockSampleState->rockStates.clear();
    for (char c : rocks) {
        if (c == 'G') {
            rockSampleState->rockStates.push_back(true);
        } else if (c == 'B') {
            rockSampleState->rockStates.push_back(false);
        } else {
            std::cerr << "Error; invalid rock state: " << c << std::endl;
        }
    }
}

