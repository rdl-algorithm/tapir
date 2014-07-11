/** @file HomecareObservation.hpp
 *
 * Defines the HomecareObservation class, which represents an observation in the Homecare POMDP.
 */
#ifndef HOMECARE_OBSERVATION_HPP_
#define HOMECARE_OBSERVATION_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "problems/shared/GridPosition.hpp"
#include "solver/abstract-problem/DiscretizedPoint.hpp"
#include "solver/abstract-problem/Observation.hpp"

#include "global.hpp"                     // for RandomGenerator

namespace homecare {
class HomecareModel;

/** A class representing an observation in the Homecare POMDP.
 *
 * This includes:
 * - an observation of the robot's own position
 * - an observation of the target's position (iff it is within once cell of the robot)
 * - a noisy observation of the general region the target is in
 * - a boolean flag representing whether or not the target is currently calling.
 */
class HomecareObservation : public solver::Point {
    friend class HomecareTextSerializer;
  public:
    /** Constructs a new HomecareObservation with the given observation data, i.e:
     * @param robotPos       The robot's observation of its own position
     * @param targetPos      The position of the target if the robot saw it within one cell, or
     *      (-1, -1) otherwise.
     * @param targetRegion   The region detected by the region sensor
     * @param call          True iff the target is currently calling.
     */
    HomecareObservation(GridPosition robotPos, GridPosition targetPos,
        int targetRegion, bool call);

    virtual ~HomecareObservation() = default;
    _NO_COPY_OR_MOVE(HomecareObservation);

    std::unique_ptr<solver::Observation> copy() const override;
    double distanceTo(solver::Observation const &otherObs) const override;
    bool equals(solver::Observation const &otherObs) const override;
    std::size_t hash() const override;
    void print(std::ostream &os) const override;

    /** Returns the position the robot has observed itself in. */
    GridPosition getRobotPos() const;

    /** Returns position of opponent if within 1 cell from robot, 
     *   otherwise returns (-1, -1) 
     */
    GridPosition getTargetPos() const;

    /** Returns the region the observation sensor considers the target to be in (noisy!) */
    int getTargetRegion() const;

    /** Returns true iff the robot observed the target calling (this is accurate). */
    bool getCall() const;

  private:
    GridPosition robotPos_;
    GridPosition targetPos_;
    int targetRegion_;
    bool call_;
};
} /* namespace homecare */
#endif /* HOMECARE_OBSERVATION_HPP_ */
