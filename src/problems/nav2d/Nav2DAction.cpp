#include "Nav2DAction.hpp"

#include <cstddef>                      // for size_t

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <sstream>
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for of vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/abstract-problem/State.hpp"             // for State

#include "Nav2DModel.hpp"

namespace nav2d {
Nav2DAction::Nav2DAction(double speed, double rotationalSpeed, long binNumber) :
    speed_(speed),
    rotationalSpeed_(rotationalSpeed),
    binNumber_(binNumber) {
}

Nav2DAction::Nav2DAction(double speed, double rotationalSpeed,
        Nav2DModel *model) :
                Nav2DAction(speed, rotationalSpeed,
                        calculateBinNumber(speed, rotationalSpeed, model)) {
}

Nav2DAction::Nav2DAction(long binNo, Nav2DModel *model) :
        Nav2DAction(getSpeed(binNo, model), getRotationalSpeed(binNo, model),
                binNo) {
}

Nav2DAction::Nav2DAction(Nav2DAction const &other) :
        Nav2DAction(other.speed_, other.rotationalSpeed_, other.binNumber_) {
}

std::unique_ptr<solver::Action> Nav2DAction::copy() const {
    return std::make_unique<Nav2DAction>(*this);
}

double Nav2DAction::distanceTo(solver::Action const &/*otherAction*/) const {
    return 0;
}

bool Nav2DAction::equals(solver::Action const &otherAction) const {
    Nav2DAction const &other = static_cast<Nav2DAction const &>(otherAction);
    return (speed_ == other.speed_
            && rotationalSpeed_ == other.rotationalSpeed_);
}

std::size_t Nav2DAction::hash() const {
    std::size_t hashValue = 0;
    abt::hash_combine(hashValue, speed_);
    abt::hash_combine(hashValue, rotationalSpeed_);
    return hashValue;
}

void Nav2DAction::print(std::ostream &os) const {
    os << "A";
    abt::printWithWidth(getBinNumber(), os, 2);
    os << ":(";
    abt::printDouble(speed_, os, 5, 3);
    os << "/";
    abt::printDouble(rotationalSpeed_, os, 6, 3,
            std::ios_base::fixed | std::ios_base::showpos);
    os << ")";
}

double Nav2DAction::getSpeed() const {
    return speed_;
}
double Nav2DAction::getRotationalSpeed() const{
    return rotationalSpeed_;
}
long Nav2DAction::getBinNumber() const {
    return binNumber_;
}

long Nav2DAction::getNumberOfBins() {
    return (SPEED_CUTOFFS.size() - 1) * (ROTATIONAL_SPEED_CUTOFFS.size() - 1);
}

double Nav2DAction::getValue(long index,
            std::vector<double> const &cutoffs) {
    if (index < 0 || index > (long)cutoffs.size() - 2) {
        debug::show_message("ERROR: Invalid index!");
        return std::numeric_limits<double>::signaling_NaN();
    }
    return (index == 0 ? cutoffs[0] : (
            index == (long)cutoffs.size() - 2 ? cutoffs[index+1] : (
                    (cutoffs[index] + cutoffs[index+1]) / 2)));
}
long Nav2DAction::getIndex(double value,
        std::vector<double> const &cutoffs) {
    double error = 0;
    if (value < cutoffs[0]) {
        error = cutoffs[0] - value;
        value = cutoffs[0];
    }
    if (value > cutoffs[cutoffs.size()-1]) {
        error = cutoffs[cutoffs.size()-1] - value;
        value = cutoffs[cutoffs.size()-1];
    }
    if (error > 0.0) {
        std::ostringstream sstr;
        sstr << "WARNING: Error of " << error << " in binning!!";
        debug::show_message(sstr.str());
    }
    for (uint16_t i = 1; i < cutoffs.size(); i++) {
        if (value <= cutoffs[i]) {
            return i-1;
        }
    }
    debug::show_message("ERROR: Invalid value!?");
    return -1;
}

std::vector<double> Nav2DAction::SPEED_CUTOFFS {0, 0.2, 0.8, 1.0 };
std::vector<double> Nav2DAction::ROTATIONAL_SPEED_CUTOFFS  {
    -1, -0.5, 0.5, 1.0};
double Nav2DAction::getSpeed(long binNo, Nav2DModel *model) {
    long index = binNo / (ROTATIONAL_SPEED_CUTOFFS.size() - 1);
    return model->getMaxSpeed() * getValue(index, SPEED_CUTOFFS);
}
double Nav2DAction::getRotationalSpeed(long binNo,
        Nav2DModel *model) {
    long index = binNo % (ROTATIONAL_SPEED_CUTOFFS.size() - 1);
    return model->getMaxRotationalSpeed() * getValue(
            index, ROTATIONAL_SPEED_CUTOFFS);
}
long Nav2DAction::calculateBinNumber(double speed,
        double rotationalSpeed, Nav2DModel *model) {
    long speedIndex = getIndex(
            speed / model->getMaxSpeed(), SPEED_CUTOFFS);
    long rotationalSpeedIndex = getIndex(
            rotationalSpeed / model->getMaxRotationalSpeed(),
            ROTATIONAL_SPEED_CUTOFFS);
    return rotationalSpeedIndex + speedIndex * (
            ROTATIONAL_SPEED_CUTOFFS.size() - 1);
}

} /* namespace nav2d */
