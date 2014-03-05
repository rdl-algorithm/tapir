#ifndef NAV2D_ACTION_HPP_
#define NAV2D_ACTION_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "solver/geometry/Action.hpp"
#include "solver/geometry/EnumeratedPoint.hpp"

namespace nav2d {
class Nav2DModel;

enum class ActionType {
    FORWARD,
//    FORWARD_1,
//    FORWARD_2,
    FORWARD_LEFT,
//    TURN_LEFT_1,
//    TURN_LEFT_2,
    FORWARD_RIGHT,
//    TURN_RIGHT_1,
//    TURN_RIGHT_2,
//    DO_NOTHING,
    END // Not an action, but must be last to count the number of actions.
};

class Nav2DAction : public solver::EnumeratedPoint {
    friend class Nav2DTextSerializer;
  public:
    Nav2DAction(ActionType type, double speed, double rotationalSpeed);
    Nav2DAction(ActionType type, Nav2DModel *model);

    virtual ~Nav2DAction() = default;
    // Copy constructor is allowed, but not others.
    Nav2DAction(Nav2DAction const &other);
    Nav2DAction(Nav2DAction &&) = delete;
    Nav2DAction &operator=(Nav2DAction const &) = delete;
    Nav2DAction &operator=(Nav2DAction &&) = delete;

    std::unique_ptr<solver::Action> copy() const override;
    double distanceTo(solver::Action const &otherAction) const override;
    bool equals(solver::Action const &otherAction) const override;
    std::size_t hash() const override;
    void print(std::ostream &os) const override;

    double getSpeed() const;
    double getRotationalSpeed() const;

    ActionType getType() const;
    long getCode() const;

  private:
    ActionType type_;
    double speed_;
    double rotationalSpeed_;
};
} /* namespace nav2d */

#endif /* NAV2D_ACTION_HPP_ */
