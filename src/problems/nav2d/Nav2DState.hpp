#ifndef NAV2D_STATE_HPP_
#define NAV2D_STATE_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/geometry/State.hpp"             // for State
#include "solver/geometry/Vector.hpp"             // for Vector

namespace nav2d {
class Nav2DState : public solver::Vector {
    friend class Nav2DTextSerializer;
  public:
    Nav2DState(GridPosition position, std::vector<bool> rockStates);

    virtual ~Nav2DState() = default;
    Nav2DState(Nav2DState const &other);
    Nav2DState(Nav2DState &&) = delete;
    virtual Nav2DState &operator=(Nav2DState const &) = delete;
    virtual Nav2DState &operator=(Nav2DState &&) = delete;

    std::unique_ptr<solver::State> copy() const override;
    double distanceTo(solver::State const &otherState) const override;
    bool equals(solver::State const &otherState) const override;
    std::size_t hash() const override;
    void print(std::ostream &os) const override;

    std::vector<double> asVector() const override;

    GridPosition getPosition() const;
    std::vector<bool> getRockStates() const;

  private:
    GridPosition position_;
    std::vector<bool> rockStates_;
};
} /* namespace nav2d */

#endif /* NAV2DSTATE_HPP_ */
