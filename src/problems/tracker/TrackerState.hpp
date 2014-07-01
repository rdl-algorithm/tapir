#ifndef TRACKERSTATE_HPP_
#define TRACKERSTATE_HPP_

#include <cstddef>                      // for size_t

#include <memory>
#include <ostream>                      // for ostream
#include <vector>

#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/abstract-problem/State.hpp"
#include "solver/abstract-problem/VectorState.hpp"

namespace tracker {
class TrackerState : public solver::VectorState {
    friend class TrackerTextSerializer;
  public:
    TrackerState(GridPosition robotPos, int robotYaw, 
        GridPosition targetPos, int targetYaw, bool _isVisible);

    virtual ~TrackerState() = default;
    TrackerState(TrackerState const &);
    TrackerState(TrackerState &&) = delete;
    virtual TrackerState &operator=(TrackerState const &) = delete;
    virtual TrackerState &operator=(TrackerState &&) = delete;

    std::unique_ptr<solver::Point> copy() const override;

    double distanceTo(solver::State const &otherState) const override;
    bool equals(solver::State const &otherState) const override;
    std::size_t hash() const;

    std::vector<double> asVector() const override;
    void print(std::ostream &os) const override;

    GridPosition getRobotPos() const;
    GridPosition getTargetPos() const;
    int getRobotYaw() const;
    int getTargetYaw() const;
    bool seesTarget() const;
  private:
    GridPosition robotPos_, targetPos_;
    int robotYaw_, targetYaw_;
    bool seesTarget_;
};
} /* namespace tracker */

// We define a hash function directly in the std namespace.
namespace std {
template<> struct hash<tracker::TrackerState> {
    std::size_t operator()(tracker::TrackerState const &state) const {
        return state.hash();
    }
};
} /* namespace std */

#endif /* TRACKERSTATE_HPP_ */
