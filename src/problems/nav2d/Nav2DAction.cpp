#include "Nav2DAction.hpp"

#include <cstddef>                      // for size_t

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for of vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/abstract-problem/State.hpp"             // for State

#include "Nav2DModel.hpp"

namespace nav2d {
Nav2DAction::Nav2DAction(ActionType type, double speed, double rotationalSpeed) :
        type_(type),
        speed_(speed),
        rotationalSpeed_(rotationalSpeed) {
}

Nav2DAction::Nav2DAction(ActionType type, Nav2DModel *model) :
        type_(type),
        speed_(0.0),
        rotationalSpeed_(0.0) {
    switch (type) {
    case ActionType::FORWARD:
        speed_ = model->maxSpeed_;
        return;
//    case ActionType::FORWARD_1:
//        speed_ = 0.6 * model->maxSpeed_;
//        return;
//    case ActionType::FORWARD_2:
//        speed_ = 0.2 * model->maxSpeed_;
//        return;
    case ActionType::LEFT:
        // speed_= model->maxSpeed_ / 2;
        rotationalSpeed_ = model->maxRotationalSpeed_;
        return;
//    case ActionType::TURN_LEFT_1:
//        rotationalSpeed_ = 0.6 * model->maxRotationalSpeed_;
//        return;
//    case ActionType::TURN_LEFT_2:
//        rotationalSpeed_ = model->maxRotationalSpeed_;
//        return;
    case ActionType::RIGHT:
        // speed_ = model->maxSpeed_ / 2;
        rotationalSpeed_ =  -model->maxRotationalSpeed_;
        return;
//    case ActionType::TURN_RIGHT_1:
//        rotationalSpeed_ = -0.6 * model->maxRotationalSpeed_;
//        return;
//    case ActionType::TURN_RIGHT_2:
//        rotationalSpeed_ = -0.2 * model->maxRotationalSpeed_;
//        return;
//    case ActionType::DO_NOTHING:
//        return;
    default:
        std::ostringstream message;
        message << "ERROR: Invalid Action Code " << getBinNumber();
        debug::show_message(message.str());
        return;
    }
}


Nav2DAction::Nav2DAction(Nav2DAction const &other) :
        Nav2DAction(other.type_, other.speed_, other.rotationalSpeed_) {
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
    os << "#" << getBinNumber() << ":" << speed_ << "/" << rotationalSpeed_;
}

double Nav2DAction::getSpeed() const {
    return speed_;
}
double Nav2DAction::getRotationalSpeed() const{
    return rotationalSpeed_;
}
ActionType Nav2DAction::getType() const {
    return type_;
}
long Nav2DAction::getBinNumber() const {
    return static_cast<long>(type_);
}

} /* namespace nav2d */
