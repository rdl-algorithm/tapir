#ifndef ROCKSAMPLE_ACTION_HPP_
#define ROCKSAMPLE_ACTION_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/Action.hpp"             // for State

namespace rocksample {
class RockSampleAction : public solver::Action {
    friend class RockSampleTextSerializer;
  public:
    enum class Code : long {
        NORTH = 0,
        EAST = 1,
        SOUTH = 2,
        WEST = 3,
        SAMPLE = 4,
        CHECK = 5
    };

    RockSampleAction(Code code);
    RockSampleAction(long rockNo);
    RockSampleAction(Code code, long rockNo);

    virtual ~RockSampleAction() = default;
    RockSampleAction(RockSampleAction const &other) = default;
    RockSampleAction(RockSampleAction &&) = default;
    virtual RockSampleAction &operator=(RockSampleAction const &) = default;
    virtual RockSampleAction &operator=(RockSampleAction &&) = default;

    std::unique_ptr<solver::Action> copy() const;

    double distanceTo(solver::Action const &otherAction) const;
    bool equals(solver::Action const &otherAction) const;
    std::size_t hash() const;
    void print(std::ostream &os) const;

    Code getCode() const;
    long getRockNo() const;
  private:
    Code code_;
    long rockNo_;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_ACTION_HPP_ */
