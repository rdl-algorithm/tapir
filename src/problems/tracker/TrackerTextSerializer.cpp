#include "TrackerTextSerializer.hpp"

#include <iostream>                     // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, basic_istream<>::__istream_type

#include "global.hpp"                     // for make_unique
#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/Observation.hpp"
#include "solver/abstract-problem/State.hpp"             // for State
#include "solver/serialization/TextSerializer.hpp"    // for TextSerializer

#include "solver/mappings/enumerated_actions.hpp"
#include "solver/mappings/discrete_observations.hpp"

#include "TrackerAction.hpp"
#include "TrackerModel.hpp"
#include "TrackerObservation.hpp"
#include "TrackerState.hpp"                 // for TrackerState

namespace solver {
class Solver;
} /* namespace solver */

namespace tracker {
TrackerTextSerializer::TrackerTextSerializer(solver::Solver *solver) :
    solver::Serializer(solver) {
}

void TrackerTextSerializer::saveState(solver::State const *state, std::ostream &os) {
    TrackerState const &trackerState = static_cast<TrackerState const &>(*state);
    os << trackerState.robotPos_.i << " " << trackerState.robotPos_.j << " "
        << trackerState.robotYaw_ << " " << trackerState.targetPos_.i << " "
        << trackerState.targetPos_.j << " " << trackerState.targetYaw_ << " "
        << (trackerState.seesTarget_ ? "T" : "_");
}

std::unique_ptr<solver::State> TrackerTextSerializer::loadState(std::istream &is) {
    long i, j;
    int robotYaw, targetYaw;
    is >> i >> j;
    GridPosition robotPos(i, j);
    is >> robotYaw;
    is >> i >> j;
    GridPosition targetPos(i, j);
    is>> targetYaw;
    std::string trackergedString;
    is >> trackergedString;
    bool seesTarget = (trackergedString == "T");
    return std::make_unique<TrackerState>(robotPos, robotYaw,
        targetPos, targetYaw, seesTarget);
}


void TrackerTextSerializer::saveObservation(solver::Observation const *obs,
        std::ostream &os) {
    if (obs == nullptr) {
        os << "()";
    } else {
        TrackerObservation const &observation = 
            static_cast<TrackerObservation const &>(*obs);
        os << "(" << observation.robotPos_.i << " " << observation.robotPos_.j;
        os << " " << observation.robotYaw_;
        os << " " << (observation.seesTarget_ ? "SEEN" : "UNSEEN") << ")";
    }
}

std::unique_ptr<solver::Observation> TrackerTextSerializer::loadObservation(
        std::istream &is) {
    std::string obsString;
    std::getline(is, obsString, '(');
    std::getline(is, obsString, ')');
    if (obsString == "") {
        return nullptr;
    }
    long i, j;
    int robotYaw;
    std::string tmpStr;
    std::istringstream(obsString) >> i >> j >> tmpStr;
    std::istringstream(obsString) >> robotYaw;
    bool seesTarget = tmpStr == "SEEN";
    return std::make_unique<TrackerObservation>(GridPosition(i, j), robotYaw, seesTarget);
}


void TrackerTextSerializer::saveAction(solver::Action const *action,
        std::ostream &os) {
    if (action == nullptr) {
        os << "NULL";
        return;
    }
    TrackerAction const &a =
            static_cast<TrackerAction const &>(*action);
    ActionType code = a.getActionType();
    switch (code) {
    case ActionType::FORWARD:
        os << "FORWARD";
        break;
    case ActionType::TURN_RIGHT:
        os << "TURN_RIGHT";
        break;
    case ActionType::TURN_LEFT:
        os << "TURN_LEFT";
        break;
    case ActionType::REVERSE:
        os << "REVERSE";
        break;
    case ActionType::WAIT:
        os << "WAIT";
        break;
    default:
        os << "ERROR-" << static_cast<long>(code);
        break;
    }
}

std::unique_ptr<solver::Action> TrackerTextSerializer::loadAction(
        std::istream &is) {
    std::string text;
    is >> text;
    if (text == "NULL") {
        return nullptr;
    } else if (text == "FORWARD") {
        return std::make_unique<TrackerAction>(ActionType::FORWARD);
    } else if (text == "TURN_RIGHT") {
        return std::make_unique<TrackerAction>(ActionType::TURN_RIGHT);
    } else if (text == "TURN_LEFT") {
        return std::make_unique<TrackerAction>(ActionType::TURN_LEFT);
    } else if (text == "REVERSE") {
        return std::make_unique<TrackerAction>(ActionType::REVERSE);
    } else if (text == "WAIT") {
        return std::make_unique<TrackerAction>(ActionType::WAIT);
    } else {
        std::string tmpStr;
        std::istringstream sstr(text);
        std::getline(sstr, tmpStr, '-');
        long code;
        sstr >> code;
        debug::show_message("ERROR: Invalid action!");
        return std::make_unique<TrackerAction>(
                static_cast<ActionType>(code));
    }
}


int TrackerTextSerializer::getActionColumnWidth(){
    return 5;
}
int TrackerTextSerializer::getTPColumnWidth() {
    return 0;
}
int TrackerTextSerializer::getObservationColumnWidth() {
    return 12;
}
} /* namespace tracker */
