/** @file RockSampleTextSerializer.cpp
 *
 * Contains the implementations of the serialization methods for RockSample.
 */
#include "RockSampleTextSerializer.hpp"

#include <iostream>                     // for operator<<, basic_ostream, basic_istream<>::__istream_type, basic_ostream<>::__ostream_type, endl
#include <string>                       // for operator>>, string
#include <vector>                       // for vector

#include "global.hpp"                     // for make_unique
#include "problems/shared/GridPosition.hpp"  // for GridPosition

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/State.hpp"
#include "solver/abstract-problem/Observation.hpp"

#include "solver/mappings/actions/enumerated_actions.hpp"
#include "solver/mappings/observations/enumerated_observations.hpp"

#include "solver/serialization/TextSerializer.hpp"    // for TextSerializer

#include "RockSampleAction.hpp"         // for RockSampleAction
#include "RockSampleObservation.hpp"    // for RockSampleObservation
#include "RockSampleState.hpp"          // for RockSampleState
#include "RockSampleModel.hpp"

namespace solver {
class Solver;
} /* namespace solver */

namespace rocksample {
void saveVector(std::vector<long> values, std::ostream &os) {
    os << "(";
    for (auto it = values.begin(); it != values.end(); it++) {
        os << *it;
        if ((it + 1) != values.end()) {
            os << ", ";
        }
    }
    os << ")";
}

std::vector<long> loadVector(std::istream &is) {
    std::vector<long> values;
    std::string tmpStr;
    std::getline(is, tmpStr, '(');
    std::getline(is, tmpStr, ')');
    std::istringstream sstr(tmpStr);
    while (std::getline(sstr, tmpStr, ',')) {
        long value;
        std::istringstream(tmpStr) >> value;
        values.push_back(value);
    }
    return values;
}

RockSampleTextSerializer::RockSampleTextSerializer(solver::Solver *solver) :
        Serializer(solver) {
}

/* ------------------ Saving change sequences -------------------- */
void RockSampleTextSerializer::saveModelChange(solver::ModelChange const &change, std::ostream &os) {
    RockSampleChange const &rsChange = static_cast<RockSampleChange const &>(change);
    os << rsChange.changeType;
    os << ": ";
    saveVector(std::vector<long> {rsChange.i0, rsChange.j0}, os);
    os << " ";
    saveVector(std::vector<long> {rsChange.i1, rsChange.j1}, os);
}


std::unique_ptr<solver::ModelChange> RockSampleTextSerializer::loadModelChange(std::istream &is) {
    std::unique_ptr<RockSampleChange> change = std::make_unique<RockSampleChange>();
    std::getline(is, change->changeType, ':');
    std::vector<long> v0 = loadVector(is);
    std::vector<long> v1 = loadVector(is);
    change->i0 = v0[0];
    change->j0 = v0[1];
    change->i1 = v1[0];
    change->j1 = v1[1];
    return std::move(change);
}

void RockSampleTextSerializer::saveState(solver::State const *state, std::ostream &os) {
    if (state == nullptr) {
        os << "NULL";
        return;
    }
    RockSampleState const &rockSampleState = static_cast<RockSampleState const &>(*state);
    os << rockSampleState.position_.i << " " << rockSampleState.position_.j << " ";
    for (bool isGood : rockSampleState.getRockStates()) {
        if (isGood) {
            os << 'G';
        } else {
            os << 'B';
        }
    }
}

std::unique_ptr<solver::State> RockSampleTextSerializer::loadState(std::istream &is) {
    std::string text;
    is >> text;
    if (text == "NULL") {
        return nullptr;
    }
    long i, j;
    std::istringstream(text) >> i;
    std::string rockString;
    std::vector<bool> rockStates;

    is >> j >> rockString;
    for (char c : rockString) {
        if (c == 'G') {
            rockStates.push_back(true);
        } else if (c == 'B') {
            rockStates.push_back(false);
        } else {
            std::ostringstream message;
            message << "ERROR: Invalid rock state: " << c;
            debug::show_message(message.str());
        }
    }
    return std::make_unique<RockSampleState>(GridPosition(i, j), rockStates);
}

void RockSampleTextSerializer::saveObservation(solver::Observation const *obs, std::ostream &os) {
    if (obs == nullptr) {
        os << "NULL";
        return;
    }
    RockSampleObservation const &observation = static_cast<RockSampleObservation const &>(*obs);
    if (observation.isEmpty()) {
        os << "NONE";
    } else if (observation.isGood_) {
        os << "GOOD";
    } else {
        os << "BAD";
    }
}

std::unique_ptr<solver::Observation> RockSampleTextSerializer::loadObservation(std::istream &is) {
    std::string text;
    is >> text;
    if (text == "NULL") {
        return nullptr;
    } else if (text == "NONE") {
        return std::make_unique<RockSampleObservation>(true, true);
    } else if (text == "GOOD") {
        return std::make_unique<RockSampleObservation>(false, true);
    } else if (text == "BAD") {
        return std::make_unique<RockSampleObservation>(false, false);
    } else {
        debug::show_message("ERROR: Invalid observation!");
        return nullptr;
    }
}

void RockSampleTextSerializer::saveAction(solver::Action const *action, std::ostream &os) {
    if (action == nullptr) {
        os << "NULL";
        return;
    }
    RockSampleAction const &a = static_cast<RockSampleAction const &>(*action);
    ActionType code = a.getActionType();
    if (code == ActionType::CHECK) {
        os << "CHECK-" << a.getRockNo();
        return;
    }
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
    case ActionType::SAMPLE:
        os << "SAMPLE";
        break;
    default:
        os << "ERROR-" << static_cast<long>(code);
        break;
    }
}

std::unique_ptr<solver::Action> RockSampleTextSerializer::loadAction(std::istream &is) {
    std::string text;
    is >> text;
    if (text == "NULL") {
        return nullptr;
    } else if (text == "NORTH") {
        return std::make_unique<RockSampleAction>(ActionType::NORTH);
    } else if (text == "EAST") {
        return std::make_unique<RockSampleAction>(ActionType::EAST);
    } else if (text == "SOUTH") {
        return std::make_unique<RockSampleAction>(ActionType::SOUTH);
    } else if (text == "WEST") {
        return std::make_unique<RockSampleAction>(ActionType::WEST);
    } else if (text == "SAMPLE") {
        return std::make_unique<RockSampleAction>(ActionType::SAMPLE);
    } else if (text.find("CHECK") != std::string::npos) {
        std::string tmpStr;
        std::istringstream sstr(text);
        std::getline(sstr, tmpStr, '-');
        long rockNo;
        sstr >> rockNo;
        return std::make_unique<RockSampleAction>(ActionType::CHECK, rockNo);
    } else {
        std::string tmpStr;
        std::istringstream sstr(text);
        std::getline(sstr, tmpStr, '-');
        long code;
        sstr >> code;
        std::ostringstream message;
        message << "ERROR: Invalid action; code " << code;
        debug::show_message(message.str());
        return std::make_unique<RockSampleAction>(static_cast<ActionType>(code));
    }
}

int RockSampleTextSerializer::getActionColumnWidth() {
    return 7;
}
int RockSampleTextSerializer::getTPColumnWidth() {
    return 0;
}
int RockSampleTextSerializer::getObservationColumnWidth() {
    return 4;
}

RockSampleBasicTextSerializer::RockSampleBasicTextSerializer(solver::Solver *solver) :
        Serializer(solver) {
}
RockSampleLegalActionsTextSerializer::RockSampleLegalActionsTextSerializer(solver::Solver *solver) :
        Serializer(solver) {
}
RockSamplePreferredActionsTextSerializer::RockSamplePreferredActionsTextSerializer(
        solver::Solver *solver) :
        Serializer(solver) {
}
} /* namespace rocksample */
