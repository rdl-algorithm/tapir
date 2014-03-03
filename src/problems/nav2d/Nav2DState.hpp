#ifndef NAV2D_STATE_HPP_
#define NAV2D_STATE_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "problems/shared/geometry/Point2D.hpp"  // for Point2D
#include "solver/geometry/State.hpp"             // for State
#include "solver/geometry/Vector.hpp"             // for Vector

#include "global.hpp"

namespace nav2d {
class Nav2DState : public solver::Vector {
    friend class Nav2DTextSerializer;
  public:
    Nav2DState(double x, double y, double direction,
            double costPerUnitDistance, double costPerRevolution);
    Nav2DState(geometry::Point2D position, double direction,
                double costPerUnitDistance, double costPerRevolution);

    virtual ~Nav2DState();
    // Copy constructor is allowed, but not others.
    Nav2DState(Nav2DState const &other);
    Nav2DState(Nav2DState &&) = delete;
    Nav2DState &operator=(Nav2DState const &) = delete;
    Nav2DState &operator=(Nav2DState &&) = delete;

    std::unique_ptr<solver::State> copy() const override;
    double distanceTo(solver::State const &otherState) const override;
    bool equals(solver::State const &otherState) const override;
    std::size_t hash() const override;
    void print(std::ostream &os) const override;
    std::vector<double> asVector() const override;

    double round(double value) const;

    geometry::Point2D getPosition() const;
    double getX() const;
    double getY() const;
    double getDirection() const;
    bool hadCollision() const;

  private:
    geometry::Point2D position_;
    double direction_;
    double costPerUnitDistance_;
    double costPerRevolution_;
};
} /* namespace nav2d */

#endif /* NAV2DSTATE_HPP_ */
