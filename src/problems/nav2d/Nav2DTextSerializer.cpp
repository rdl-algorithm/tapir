#include "Nav2DTextSerializer.hpp"

#include <iostream>                     // for operator<<, basic_ostream, basic_istream<>::__istream_type, basic_ostream<>::__ostream_type, endl
#include <string>                       // for operator>>, string
#include <vector>                       // for vector

#include "global.hpp"                     // for make_unique
#include "problems/shared/GridPosition.hpp"  // for GridPosition

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/State.hpp"
#include "solver/abstract-problem/Observation.hpp"

#include "solver/mappings/discretized_actions.hpp"
#include "solver/mappings/enumerated_observations.hpp"

#include "solver/serialization/TextSerializer.hpp"    // for TextSerializer

#include "Nav2DAction.hpp"         // for Nav2DAction
#include "Nav2DObservation.hpp"    // for Nav2DObservation
#include "Nav2DModel.hpp"
#include "Nav2DState.hpp"          // for Nav2DState

namespace solver {
class Solver;
} /* namespace solver */

namespace nav2d {
Nav2DTextSerializer::Nav2DTextSerializer(solver::Solver *solver) :
    solver::Serializer(solver) {
}

void Nav2DTextSerializer::saveState(solver::State const *state,
        std::ostream &os) {
    if (state == nullptr) {
        os << "()";
        return;
    }
    Nav2DState const &navState =
        static_cast<Nav2DState const &>(*state);
    os << "(";
    abt::printDouble(navState.getX(), os, 10, 7);
    os << " ";
    abt::printDouble(navState.getY(), os, 10, 7);
    os << "):";
    abt::printDouble(navState.getDirection(), os, 10, 7,
            std::ios_base::fixed | std::ios_base::showpos);
}

std::unique_ptr<solver::State> Nav2DTextSerializer::loadState(
        std::istream &is) {
    std::string tmpStr;
    std::getline(is, tmpStr, '(');
    std::getline(is, tmpStr, ')');
    if (tmpStr == "") {
        return nullptr;
    }
    double x, y, direction;
    std::istringstream(tmpStr) >> x >> y;
    std::getline(is, tmpStr, ':');
    is >> direction;
    Nav2DModel const &model = dynamic_cast<Nav2DModel const &>(*model_);
    return std::make_unique<Nav2DState>(x, y, direction,
            model.costPerUnitDistance_, model.costPerRevolution_);
}

void Nav2DTextSerializer::saveAction(solver::Action const *action,
        std::ostream &os) {
    if (action == nullptr) {
        os << "A:()";
    } else {
        Nav2DAction const &a = static_cast<Nav2DAction const &>(*action);
        os << "A" << a.getBinNumber() << ":(";
        abt::printDouble(a.speed_, os, 5, 3);
        os << "/";
        abt::printDouble(a.rotationalSpeed_, os, 6, 3,
                std::ios_base::fixed | std::ios_base::showpos);
        os << ")";
    }
}

std::unique_ptr<solver::Action> Nav2DTextSerializer::loadAction(
        std::istream &is) {
    std::string tmpStr;
    // The action code lies between 'A' and ':'
    std::getline(is, tmpStr, 'A');
    std::getline(is, tmpStr, ':');
    // No code means no action;
    if (tmpStr.find_first_not_of(' ') == std::string::npos) {
        std::getline(is, tmpStr, ')');
        return nullptr;
    }
    double speed, rotationalSpeed;
    std::getline(is, tmpStr, '(');
    std::getline(is, tmpStr, '/');
    std::istringstream(tmpStr) >> speed;
    std::getline(is, tmpStr, ')');
    std::istringstream(tmpStr) >> rotationalSpeed;
    return std::make_unique<Nav2DAction>(speed, rotationalSpeed,
            dynamic_cast<Nav2DModel *>(model_));
}

void Nav2DTextSerializer::saveTransitionParameters(
        solver::TransitionParameters const *tp, std::ostream &os) {
    os << "T:(";
    if (tp != nullptr) {
        Nav2DTransition const &tp2 = static_cast<Nav2DTransition const &>(*tp);
        abt::printDouble(tp2.speed, os, 9, 7);
        os << "/";
        abt::printDouble(tp2.rotationalSpeed, os, 10, 7,
                std::ios_base::fixed | std::ios_base::showpos);
        os << " " << tp2.moveRatio << " ";
        if (tp2.reachedGoal) {
            os << "G";
        }
        if (tp2.hadCollision) {
            os << "C";
        }
        if (tp2.hitBounds) {
            os << "B";
        }
    }
    os << ")";
}

std::unique_ptr<solver::TransitionParameters>
Nav2DTextSerializer::loadTransitionParameters(
        std::istream &is) {
    std::string tmpStr;
    std::getline(is, tmpStr, '(');
    std::getline(is, tmpStr, ')');
    if (tmpStr == "") {
        return nullptr;
    }
    std::unique_ptr<Nav2DTransition> tp(
            std::make_unique<Nav2DTransition>());
    std::istringstream sstr(tmpStr);
    std::string tmpStr2;
    std::getline(sstr, tmpStr2, '/');
    std::istringstream(tmpStr2) >> tp->speed;
    sstr >> tp->rotationalSpeed;
    sstr >> tp->moveRatio;
    sstr >> tmpStr2;
    for (char c : tmpStr2) {
        if (c == 'G') {
            tp->reachedGoal = true;
        } else if (c == 'C') {
            tp->hadCollision = true;
        } else if (c == 'B')  {
            tp->hitBounds = true;
        }
    }
    return std::move(tp);
}

void Nav2DTextSerializer::saveObservation(solver::Observation const *obs,
        std::ostream &os) {
    os << "O:[";
    if (obs != nullptr) {
        Nav2DObservation const &observation =
                static_cast<Nav2DObservation const &>(*obs);
        if (observation.isEmpty()) {
            os << "()";
        } else {
            saveState(observation.state_.get(), os);
        }
    }
    os << "]";
}

std::unique_ptr<solver::Observation> Nav2DTextSerializer::loadObservation(
        std::istream &is) {
    std::string tmpStr;
    std::getline(is, tmpStr, '[');
    std::getline(is, tmpStr, ']');
    if (tmpStr == "") {
        return nullptr;
    } else if (tmpStr == "()") {
        return std::make_unique<Nav2DObservation>();
    }
    std::istringstream sstr(tmpStr);
    return std::make_unique<Nav2DObservation>(static_cast<Nav2DState const &>(
            *loadState(sstr)));
}


int Nav2DTextSerializer::getActionColumnWidth(){
    return 17;
}
int Nav2DTextSerializer::getTPColumnWidth() {
    return 28;
}
int Nav2DTextSerializer::getObservationColumnWidth() {
    return 6;
}
} /* namespace nav2d */
