#include "HomecareTextSerializer.hpp"

#include <iostream>                     // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, basic_istream<>::__istream_type

#include "global.hpp"                     // for make_unique
#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/Observation.hpp"
#include "solver/abstract-problem/State.hpp"             // for State
#include "solver/serialization/TextSerializer.hpp"    // for TextSerializer

#include "solver/mappings/actions/enumerated_actions.hpp"
#include "solver/mappings/observations/discrete_observations.hpp"

#include "HomecareAction.hpp"
#include "HomecareModel.hpp"
#include "HomecareObservation.hpp"
#include "HomecareState.hpp"                 // for HomecareState

namespace solver {
class Solver;
} /* namespace solver */

namespace homecare {

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

HomecareTextSerializer::HomecareTextSerializer(solver::Solver *solver) :
    solver::Serializer(solver) {
}

/* ------------------ Saving change sequences -------------------- */
void HomecareTextSerializer::saveModelChange(solver::ModelChange const &change, std::ostream &os) {
    HomecareChange const &homecareChange = static_cast<HomecareChange const &>(change);
    os << homecareChange.changeType;
    os << ": ";
    saveVector(std::vector<long> {homecareChange.i0, homecareChange.j0}, os);
    os << " ";
    saveVector(std::vector<long> {homecareChange.i1, homecareChange.j1}, os);
}

std::unique_ptr<solver::ModelChange> HomecareTextSerializer::loadModelChange(std::istream &is) {
    std::unique_ptr<HomecareChange> change = std::make_unique<HomecareChange>();
    std::getline(is, change->changeType, ':');
    std::vector<long> v0 = loadVector(is);
    std::vector<long> v1 = loadVector(is);
    change->i0 = v0[0];
    change->j0 = v0[1];
    change->i1 = v1[0];
    change->j1 = v1[1];
    return std::move(change);
}

void HomecareTextSerializer::saveState(solver::State const *state, std::ostream &os) {
    HomecareState const &homecareState = static_cast<HomecareState const &>(*state);
    os << homecareState.robotPos_.i << " " << homecareState.robotPos_.j << " "
       << homecareState.targetPos_.i << " " << homecareState.targetPos_.j << " "
       << (homecareState.call_ ? "C" : "_");
}

std::unique_ptr<solver::State> HomecareTextSerializer::loadState(std::istream &is) {
    long i, j;
    is >> i >> j;
    GridPosition robotPos(i, j);
    is >> i >> j;
    GridPosition targetPos(i, j);
    std::string callString;
    is >> callString;
    bool call = (callString == "C");
    return std::make_unique<HomecareState>(robotPos, targetPos, call);
}

void HomecareTextSerializer::saveObservation(solver::Observation const *obs,
        std::ostream &os) {
    if (obs == nullptr) {
        os << "()";
    } else {
        HomecareObservation const &observation = static_cast<HomecareObservation const &>(
                *obs);
        os << "(" << observation.robotPos_.i << " " << observation.robotPos_.j;
        os << " " << observation.targetPos_.i << " " << observation.targetPos_.j;
        os << " " << observation.targetRegion_;
        os << " " << (observation.call_ ? "C" : "_") << ")";
    }
}

std::unique_ptr<solver::Observation> HomecareTextSerializer::loadObservation(
        std::istream &is) {
    std::string obsString;
    std::getline(is, obsString, '(');
    std::getline(is, obsString, ')');
    if (obsString == "") {
        return nullptr;
    }
    long ri, rj, ti, tj, region;
    std::string tmpStr;
    std::istringstream(obsString) >> ri >> rj >> ti >> tj >> region >> tmpStr;
    bool call = tmpStr == "C";
    return std::make_unique<HomecareObservation>(
        GridPosition(ri, rj), GridPosition(ti, tj), region, call);
}


void HomecareTextSerializer::saveAction(solver::Action const *action,
        std::ostream &os) {
    if (action == nullptr) {
        os << "NULL";
        return;
    }
    HomecareAction const &a =
            static_cast<HomecareAction const &>(*action);
    ActionType code = a.getActionType();
    switch (code) {
    case ActionType::NORTH:
        os << "NORTH";
        break;
    case ActionType::NORTH_EAST:
        os << "NORTH_EAST";
        break;
    case ActionType::EAST:
        os << "EAST";
        break;
    case ActionType::SOUTH_EAST:
        os << "SOUTH_EAST";
        break;
    case ActionType::SOUTH:
        os << "SOUTH";
        break;
    case ActionType::SOUTH_WEST:
        os << "SOUTH_WEST";
        break;
    case ActionType::WEST:
        os << "WEST";
        break;
    case ActionType::NORTH_WEST:
        os << "NORTH_WEST";
        break;
    case ActionType::WAIT:
        os << "WAIT";
        break;
    default:
        os << "ERROR-" << static_cast<long>(code);
    }
}

std::unique_ptr<solver::Action> HomecareTextSerializer::loadAction(
        std::istream &is) {
    std::string text;
    is >> text;
    if (text == "NULL") {
        return nullptr;
    } else if (text == "NORTH") {
        return std::make_unique<HomecareAction>(ActionType::NORTH);
    } else if (text == "NORTH_EAST") {
        return std::make_unique<HomecareAction>(ActionType::NORTH_EAST);
    } else if (text == "EAST") {
        return std::make_unique<HomecareAction>(ActionType::EAST);
    } else if (text == "SOUTH_EAST") {
        return std::make_unique<HomecareAction>(ActionType::SOUTH_EAST);
    } else if (text == "SOUTH") {
        return std::make_unique<HomecareAction>(ActionType::SOUTH);
    } else if (text == "SOUTH_WEST") {
        return std::make_unique<HomecareAction>(ActionType::SOUTH_WEST);
    } else if (text == "WEST") {
        return std::make_unique<HomecareAction>(ActionType::WEST);
    } else if (text == "NORTH_WEST") {
        return std::make_unique<HomecareAction>(ActionType::NORTH_WEST);
    } else if (text == "WAIT") {
        return std::make_unique<HomecareAction>(ActionType::WAIT);
    } else {
        std::string tmpStr;
        std::istringstream sstr(text);
        std::getline(sstr, tmpStr, '-');
        long code;
        sstr >> code;
        debug::show_message("ERROR: Invalid action!");
        return std::make_unique<HomecareAction>(
                static_cast<ActionType>(code));
    }
}


int HomecareTextSerializer::getActionColumnWidth(){
    return 10;
}
int HomecareTextSerializer::getTPColumnWidth() {
    return 0;
}
int HomecareTextSerializer::getObservationColumnWidth() {
    return 13;
}
} /* namespace homecare */
