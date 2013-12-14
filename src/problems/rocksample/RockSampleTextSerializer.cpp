#include "RockSampleTextSerializer.hpp"

#include <iostream>                     // for operator<<, basic_ostream, basic_istream<>::__istream_type, basic_ostream<>::__ostream_type, cerr, endl
#include <string>                       // for operator>>, string
#include <vector>                       // for vector

#include "defs.hpp"                     // for make_unique
#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/State.hpp"             // for State
#include "solver/TextSerializer.hpp"    // for TextSerializer

#include "RockSampleState.hpp"          // for RockSampleState

namespace solver {
class Solver;
} /* namespace solver */

namespace rocksample {
RockSampleTextSerializer::RockSampleTextSerializer() :
    solver::TextSerializer() {
}

RockSampleTextSerializer::RockSampleTextSerializer(solver::Solver *solver) :
    solver::TextSerializer(solver) {
}

void RockSampleTextSerializer::saveState(solver::State &state,
        std::ostream &os) {
    RockSampleState *rockSampleState =
        static_cast<RockSampleState *>(&state);
    os << rockSampleState->position_.i << " " << rockSampleState->position_.j
       << " ";
    for (bool isGood : rockSampleState->getRockStates()) {
        if (isGood) {
            os << 'G';
        } else {
            os << 'B';
        }
    }
}

std::unique_ptr<solver::State> RockSampleTextSerializer::loadState(
        std::istream &is) {
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
} /* namespace rocksample */
