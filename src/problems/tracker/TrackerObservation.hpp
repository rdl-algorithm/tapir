#ifndef TRACKER_OBSERVATION_HPP_
#define TRACKER_OBSERVATION_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "problems/shared/GridPosition.hpp"
#include "solver/abstract-problem/DiscretizedPoint.hpp"
#include "solver/abstract-problem/Observation.hpp"

#include "global.hpp"                     // for RandomGenerator

namespace tracker {
class TrackerModel;

class TrackerObservation : public solver::Point {
    friend class TrackerTextSerializer;
  public:
    TrackerObservation(GridPosition robotPos, int robotYaw, bool seesTarget = false);

    virtual ~TrackerObservation() = default;
    _NO_COPY_OR_MOVE(TrackerObservation);

    std::unique_ptr<solver::Observation> copy() const override;
    double distanceTo(solver::Observation const &otherObs) const override;
    bool equals(solver::Observation const &otherObs) const override;
    std::size_t hash() const override;
    void print(std::ostream &os) const override;

    GridPosition getRobotPos() const;
    int getRobotYaw() const;

    bool seesTarget() const;

  private:
    GridPosition robotPos_;
    int robotYaw_;
    bool seesTarget_;
};
} /* namespace tracker */
#endif /* TRACKER_OBSERVATION_HPP_ */
