#ifndef NAV2D_ACTION_HPP_
#define NAV2D_ACTION_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "solver/geometry/Action.hpp"
#include "solver/geometry/EnumeratedPoint.hpp"             // for EnumeratedPoint

namespace nav2d {
class Nav2DAction : public solver::EnumeratedPoint {
    friend class Nav2DTextSerializer;
  public:
    Nav2DAction(double speed, double rotationalSpeed);

    virtual ~Nav2DAction() = default;
    _NO_COPY_OR_MOVE(Nav2DAction);

    std::unique_ptr<solver::Action> copy() const override;
    double distanceTo(solver::Action const &otherAction) const override;
    void print(std::ostream &os) const override;

    long getCode() const override;

    ActionType getActionType() const;
    long getRockNo() const;

    enum ActionNumbering : int {
        NORTH = 0,
        NORTHEAST = 1,
        EAST = 2,
        SOUTHEAST = 3,
        SOUTH = 4,
        SOUTHWEST = 5,
        WEST = 6,
        NORTHWEST = 7
    };
  private:
    const static Nav2DAction ACTIONS[];
    double speed;
    double rotationalSpeed;
};
} /* namespace nav2d */

#endif /* NAV2D_ACTION_HPP_ */
