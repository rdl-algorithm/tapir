/** @file TagObservation.hpp
 *
 * Defines the TagObservation class, which represents an observation in the Tag POMDP.
 */
#ifndef TAG_OBSERVATION_HPP_
#define TAG_OBSERVATION_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "global.hpp"                     // for RandomGenerator

#include "problems/shared/GridPosition.hpp"
#include "solver/abstract-problem/DiscretizedPoint.hpp"
#include "solver/abstract-problem/Observation.hpp"

namespace tag {
class TagModel;

/** A class representing an observation in the Tag POMDP.
 *
 * This includes an observation of the robot's own position, and a boolean flag representing
 * whether or not the robot sees the opponent (and hence is on the same grid square).
 */
class TagObservation : public solver::Point {
    friend class TagTextSerializer;
  public:
    /** Constructs a new TagObservation for the given robot position; seesOpponent should be true
     * iff the robot sees the opponent due to being on the same square.
     */
    TagObservation(GridPosition myPosition, bool seesOpponent = false);

    virtual ~TagObservation() = default;
    _NO_COPY_OR_MOVE(TagObservation);

    std::unique_ptr<solver::Observation> copy() const override;
    double distanceTo(solver::Observation const &otherObs) const override;
    bool equals(solver::Observation const &otherObs) const override;
    std::size_t hash() const override;
    void print(std::ostream &os) const override;

    /** Returns the position the robot has observed itself in. */
    GridPosition getPosition() const;
    /** Returns true iff the robot sees the opponent in the same square it is in. */
    bool seesOpponent() const;

  private:
    /** The position the robot sees itself in. */
    GridPosition position_;
    /** True iff the robot sees the opponent in the same square it is in. */
    bool seesOpponent_;
};
} /* namespace tag */
#endif /* TAG_OBSERVATION_HPP_ */
