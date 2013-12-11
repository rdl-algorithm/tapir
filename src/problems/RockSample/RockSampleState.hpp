#ifndef ROCKSAMPLESTATE_HPP
#define ROCKSAMPLESTATE_HPP

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "problems/GridPosition.hpp"    // for GridPosition
#include "solver/State.hpp"             // for State

class RockSampleState : public State {
    friend class RockSampleTextSerializer;
  public:
    RockSampleState(GridPosition position, std::vector<bool> rockStates);

    virtual ~RockSampleState() = default;
    RockSampleState(RockSampleState const &other);
    RockSampleState(RockSampleState &&) = delete;
    virtual RockSampleState &operator=(RockSampleState const &) = delete;
    virtual RockSampleState &operator=(RockSampleState &&) = delete;

    GridPosition getPosition() const {
        return position;
    }

    std::vector<bool> getRockStates() const {
        return rockStates;
    }

    double distanceTo(State const &otherState) const;
    bool equals(State const &otherState) const;
    std::size_t hash() const;
    void print(std::ostream &os) const;

  private:
    GridPosition position;
    std::vector<bool> rockStates;
};
#endif /* ROCKSAMPLESTATE_HPP */
