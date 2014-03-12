#include "Nav2DTextSerializer.hpp"

#include <iostream>                     // for operator<<, basic_ostream, basic_istream<>::__istream_type, basic_ostream<>::__ostream_type, endl
#include <string>                       // for operator>>, string
#include <vector>                       // for vector

#include "global.hpp"                     // for make_unique
#include "problems/shared/GridPosition.hpp"  // for GridPosition

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/State.hpp"
#include "solver/abstract-problem/Observation.hpp"

#include "solver/mappings/enumerated_actions.hpp"
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

void Nav2DTextSerializer::saveDouble(double value, std::ostream &os,
        bool showpos, int precision) {
    std::streamsize oldPrecision = os.precision(precision);
    std::streamsize width = os.width(precision + 3);
    std::ios_base::fmtflags flags = std::ios_base::fixed;
    if (showpos) {
        flags |= std::ios_base::showpos;
    }
    flags = os.flags(flags);
    os <<  value;
    os.flags(flags);
    os.width(width);
    os.precision(oldPrecision);
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
    saveDouble(navState.getX(), os);
    os << " ";
    saveDouble(navState.getY(), os);
    os << "):";
    saveDouble(navState.getDirection(), os, true);
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
        os << "NULL";
        return;
    }
    Nav2DAction const &a = static_cast<Nav2DAction const &>(*action);
    os << "#" << a.getBinNumber() << ": ";
    saveDouble(a.speed_, os, false, 1);
    os << "/";
    saveDouble(a.rotationalSpeed_, os, true, 3);
}

std::unique_ptr<solver::Action> Nav2DTextSerializer::loadAction(
        std::istream &is) {
    std::string tmpStr;
    is >> tmpStr;
    if (tmpStr == "NULL") {
        return nullptr;
    }
    long code;
    double speed, rotationalSpeed;
    std::string tmpStr2;
    {
        std::istringstream sstr(tmpStr);
        std::getline(sstr, tmpStr2, '#');
        // The action code lies between '#' and ':'
        std::getline(sstr, tmpStr2, ':');
        std::istringstream(tmpStr2) >> code;
    }

    std::getline(is, tmpStr2, '/');
    std::istringstream(tmpStr2) >> speed;
    is >> rotationalSpeed;
    return std::make_unique<Nav2DAction>(static_cast<ActionType>(code),
            speed, rotationalSpeed);
}

void Nav2DTextSerializer::saveTransitionParameters(
        solver::TransitionParameters const *tp, std::ostream &os) {
    os << "T:(";
    if (tp != nullptr) {
        Nav2DTransition const &tp2 = static_cast<Nav2DTransition const &>(*tp);
        saveDouble(tp2.speed, os);
        os << "/";
        saveDouble(tp2.rotationalSpeed, os, true);
        os << " " << tp2.moveRatio << " ";
        if (tp2.reachedGoal) {
            os << "G";
        } else if (tp2.hadCollision) {
            os << "C";
        } else {
            os << "_";
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

} /* namespace nav2d */
