#ifndef NAV2DSTATE_HPP_
#define NAV2DSTATE_HPP_

#include <cstddef>                      // for size_t

#include <memory>
#include <ostream>                      // for ostream
#include <vector>

#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/State.hpp"
#include "solver/Vector.hpp"             // for State

namespace nav2d {
class Nav2DState : public solver::Vector {
    friend class Nav2DTextSerializer;
  public:
    Nav2DState(double x, double y, double theta);

    virtual ~Nav2DState() = default;
    Nav2DState(Nav2DState const &);
    Nav2DState(Nav2DState &&) = delete;
    virtual Nav2DState &operator=(Nav2DState const &) = delete;
    virtual Nav2DState &operator=(Nav2DState &&) = delete;

    std::unique_ptr<solver::Point> copy() const;

    double distanceTo(solver::State const &otherState) const;
    bool equals(solver::State const &otherState) const;
    std::size_t hash() const;

    std::vector<double> asVector() const;
    void print(std::ostream &os) const;

    double getX() const {
        return x_;
    }

    double getY() const {
        return y_;
    }

    double getNumTurns() const {
        return numTurns_;
    }

  private:
    double x_;
    double y_;
    double numTurns_;
};
} /* namespace nav2d */

#endif /* NAV2DSTATE_HPP_ */
