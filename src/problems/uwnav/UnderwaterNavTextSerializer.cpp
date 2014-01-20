#include "UnderwaterNavTextSerializer.hpp"

#include <iostream>                     // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, basic_istream<>::__istream_type

#include "defs.hpp"                     // for make_unique
#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/State.hpp"             // for State
#include "solver/TextSerializer.hpp"    // for TextSerializer

#include "UnderwaterNavState.hpp"                 // for UnderwaterNavState

namespace solver {
class Solver;
} /* namespace solver */

namespace uwnav {
UnderwaterNavTextSerializer::UnderwaterNavTextSerializer() :
    solver::TextSerializer() {
}

UnderwaterNavTextSerializer::UnderwaterNavTextSerializer(solver::Solver *solver) :
    solver::TextSerializer(solver) {
}

void UnderwaterNavTextSerializer::saveState(solver::State &state, std::ostream &os) {
    UnderwaterNavState *uwState = static_cast<uwState *>(&state);
    os << uwState->position_.i << uwState->position_.j;
}

std::unique_ptr<solver::State> UnderwaterNavTextSerializer::loadState(std::istream &is) {
    long i, j;
    is >> i >> j;
    return std::make_unique<UnderwaterNavState>(GridPosition(i, j));
}
} /* namespace uwnav */
