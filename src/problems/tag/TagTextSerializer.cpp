#include "TagTextSerializer.hpp"

#include <iostream>                     // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, basic_istream<>::__istream_type

#include "global.hpp"                     // for make_unique
#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/geometry/Action.hpp"
#include "solver/geometry/Observation.hpp"
#include "solver/geometry/State.hpp"             // for State
#include "solver/serialization/TextSerializer.hpp"    // for TextSerializer

#include "solver/mappings/enumerated_actions.hpp"
#include "solver/mappings/discrete_observations_map.hpp"

#include "TagAction.hpp"
#include "TagModel.hpp"
#include "TagObservation.hpp"
#include "TagState.hpp"                 // for TagState

namespace solver {
class Solver;
} /* namespace solver */

namespace tag {
TagTextSerializer::TagTextSerializer(solver::Solver *solver) :
    solver::Serializer(solver) {
}

void TagTextSerializer::saveState(solver::State const *state, std::ostream &os) {
    TagState const &tagState = static_cast<TagState const &>(*state);
    os << tagState.robotPos_.i << " " << tagState.robotPos_.j << " "
       <<tagState.opponentPos_.i << " " << tagState.opponentPos_.j << " "
       << tagState.isTagged_;
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


void TagTextSerializer::saveObservation(solver::Observation const *obs,
        std::ostream &os) {
    if (obs == nullptr) {
        os << "()";
        return;
    }
    TagObservation const &observation = static_cast<TagObservation const &>(
            *obs);
    os << "(" << observation.position_.i << " " << observation.position_.j;
    os << " " << (observation.seesOpponent_ ? "SEEN" : "UNSEEN") << ")";
}

std::unique_ptr<solver::Observation> TagTextSerializer::loadObservation(
        std::istream &is) {
    std::string obsString;
    std::getline(is, obsString, '(');
    std::getline(is, obsString, ')');
    if (obsString == "") {
        return nullptr;
    }
    long i, j;
    std::string tmpStr;
    std::istringstream(obsString) >> i >> j >> tmpStr;
    bool seesOpponent = tmpStr == "SEEN";
    return std::make_unique<TagObservation>(dynamic_cast<TagModel *>(
            model_), GridPosition(i, j), seesOpponent);
}


void TagTextSerializer::saveAction(solver::Action const *action,
        std::ostream &os) {
    if (action == nullptr) {
        os << "NULL";
        return;
    }
    TagAction const &a =
            static_cast<TagAction const &>(*action);
    ActionType code = a.getActionType();
    switch (code) {
    case ActionType::NORTH:
        os << "NORTH";
        break;
    case ActionType::EAST:
        os << "EAST";
        break;
    case ActionType::SOUTH:
        os << "SOUTH";
        break;
    case ActionType::WEST:
        os << "WEST";
        break;
    case ActionType::TAG:
        os << "TAG";
        break;
    default:
        os << "ERROR-" << static_cast<long>(code);
        break;
    }
}

std::unique_ptr<solver::Action> TagTextSerializer::loadAction(
        std::istream &is) {
    std::string text;
    is >> text;
    if (text == "NULL") {
        return nullptr;
    } else if (text == "NORTH") {
        return std::make_unique<TagAction>(ActionType::NORTH);
    } else if (text == "EAST") {
        return std::make_unique<TagAction>(ActionType::EAST);
    } else if (text == "SOUTH") {
        return std::make_unique<TagAction>(ActionType::SOUTH);
    } else if (text == "WEST") {
        return std::make_unique<TagAction>(ActionType::WEST);
    } else if (text == "TAG") {
        return std::make_unique<TagAction>(ActionType::TAG);
    } else {
        std::string tmpStr;
        std::istringstream sstr(text);
        std::getline(sstr, tmpStr, '-');
        long code;
        sstr >> code;
        debug::show_message("ERROR: Invalid action!");
        return std::make_unique<TagAction>(
                static_cast<ActionType>(code));
    }
}
} /* namespace tag */
