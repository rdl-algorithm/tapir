#include "TagTextSerializer.hpp"

#include <iostream>                     // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, basic_istream<>::__istream_type

#include "global.hpp"                     // for make_unique
#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/Observation.hpp"
#include "solver/abstract-problem/State.hpp"             // for State
#include "solver/serialization/TextSerializer.hpp"    // for TextSerializer

#include "solver/mappings/actions/enumerated_actions.hpp"
#include "solver/mappings/observations/discrete_observations.hpp"

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

/* ------------------ Saving change sequences -------------------- */
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
void TagTextSerializer::saveModelChange(solver::ModelChange const &change, std::ostream &os) {
    TagChange const &tagChange = static_cast<TagChange const &>(change);
    os << tagChange.changeType;
    os << ": ";
    saveVector(std::vector<long> {(long)tagChange.i0, (long)tagChange.j0}, os);
    os << " ";
    saveVector(std::vector<long> {(long)tagChange.i1, (long)tagChange.j1}, os);
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
std::unique_ptr<solver::ModelChange> TagTextSerializer::loadModelChange(std::istream &is) {
    std::unique_ptr<TagChange> change = std::make_unique<TagChange>();
    std::getline(is, change->changeType, ':');
    std::vector<long> v0 = loadVector(is);
    std::vector<long> v1 = loadVector(is);
    change->i0 = v0[0];
    change->j0 = v0[1];
    change->i1 = v1[0];
    change->j1 = v1[1];
    return std::move(change);
}

void TagTextSerializer::saveState(solver::State const *state, std::ostream &os) {
    TagState const &tagState = static_cast<TagState const &>(*state);
    os << tagState.robotPos_.i << " " << tagState.robotPos_.j << " "
       << tagState.opponentPos_.i << " " << tagState.opponentPos_.j << " "
       << (tagState.isTagged_ ? "T" : "_");
}

std::unique_ptr<solver::State> TagTextSerializer::loadState(std::istream &is) {
    long i, j;
    is >> i >> j;
    GridPosition robotPos(i, j);
    is >> i >> j;
    GridPosition opponentPos(i, j);
    std::string taggedString;
    is >> taggedString;
    bool isTagged = (taggedString == "T");
    return std::make_unique<TagState>(robotPos, opponentPos, isTagged);
}


void TagTextSerializer::saveObservation(solver::Observation const *obs,
        std::ostream &os) {
    if (obs == nullptr) {
        os << "()";
    } else {
        TagObservation const &observation = static_cast<TagObservation const &>(
                *obs);
        os << "(" << observation.position_.i << " " << observation.position_.j;
        os << " " << (observation.seesOpponent_ ? "SEEN" : "UNSEEN") << ")";
    }
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
    return std::make_unique<TagObservation>(GridPosition(i, j), seesOpponent);
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


int TagTextSerializer::getActionColumnWidth(){
    return 5;
}
int TagTextSerializer::getTPColumnWidth() {
    return 0;
}
int TagTextSerializer::getObservationColumnWidth() {
    return 12;
}
} /* namespace tag */
