#ifndef NAV2D_OBSERVATION_HPP_
#define NAV2D_OBSERVATION_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "solver/geometry/EnumeratedPoint.hpp"
#include "solver/geometry/Observation.hpp"

#include "global.hpp"                     // for RandomGenerator

namespace nav2d {
class Nav2DObservation : public solver::Point {
    friend class Nav2DTextSerializer;
  public:
    Nav2DObservation();
    Nav2DObservation(Nav2DState const &state);
    Nav2DObservation(double x, double y, double direction);

    virtual ~Nav2DObservation() = default;
    _NO_COPY_OR_MOVE(Nav2DObservation);

    std::unique_ptr<solver::Observation> copy() const override;
    double distanceTo(solver::Observation const &otherObs) const override;
    bool equals(solver::Observation const &otherObs) const override;
    std::size_t hash() const override;
    void print(std::ostream &os) const override;

    bool isEmpty() const;
    Nav2DState const *getState() const;
    double getX() const;
    double getY() const;
    double getDirection() const;

  private:
    std::unique_ptr<Nav2DState> state_;
};
} /* namespace nav2d */

#endif /* NAV2D_OBSERVATION_HPP_ */
