#ifndef ROCKSAMPLE_STATE_HPP_
#define ROCKSAMPLE_STATE_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/State.hpp"             // for State
#include "solver/Vector.hpp"             // for Vector

namespace rocksample {
class RockSampleState : public solver::Vector {
    friend class RockSampleTextSerializer;
  public:
    RockSampleState(GridPosition position, std::vector<bool> rockStates);

    virtual ~RockSampleState() = default;
    RockSampleState(RockSampleState const &other);
    RockSampleState(RockSampleState &&) = delete;
    virtual RockSampleState &operator=(RockSampleState const &) = delete;
    virtual RockSampleState &operator=(RockSampleState &&) = delete;

    std::unique_ptr<solver::State> copy() const;

    GridPosition getPosition() const {
        return position_;
    }

    std::vector<bool> getRockStates() const {
        return rockStates_;
    }

    double distanceTo(solver::State const &otherState) const;
    bool equals(solver::State const &otherState) const;
    std::size_t hash() const;
    void print(std::ostream &os) const;

    std::vector<double> asVector() const;

  private:
    GridPosition position_;
    std::vector<bool> rockStates_;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLESTATE_HPP_ */
