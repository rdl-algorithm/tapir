/** @file RockSampleObservation.hpp
 *
 * Defines the RockSampleObservation class, which represents an observation in the RockSample POMDP.
 */
#ifndef ROCKSAMPLE_OBSERVATION_HPP_
#define ROCKSAMPLE_OBSERVATION_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "solver/abstract-problem/DiscretizedPoint.hpp"
#include "solver/abstract-problem/Observation.hpp"

#include "global.hpp"                     // for RandomGenerator

namespace rocksample {
/** A class representing an observation in the RockSample POMDP.
 *
 * This is represented by two boolean flags; one for whether or not any observation was made,
 * and one for whether the rock was seen as good or bad if an observation was, in fact, made.
 *
 * This class also implements solver::DiscretizedPoint so that the solver can use a simplistic
 * enumerated observation mapping approach (EnumeratedObservationPool) to store the possible
 * observations from each ActionNode.
 */
class RockSampleObservation : public solver::DiscretizedPoint {
    friend class RockSampleTextSerializer;

  public:
    /** Constructs a new, empty observation. */
    RockSampleObservation();
    /** Constructs a non-empty observation, which is good iff _isGood is true. */
    RockSampleObservation(bool _isGood);
    /** Constructs a new observation with the given values for whether it is empty, and whether it
     * is good.
     */
    RockSampleObservation(bool _isEmpty, bool _isGood);
    /** Constructs a new observation from the given integer code. */
    RockSampleObservation(long code);

    virtual ~RockSampleObservation() = default;
    _NO_COPY_OR_MOVE(RockSampleObservation);

    std::unique_ptr<solver::Observation> copy() const override;
    double distanceTo(solver::Observation const &otherObs) const override;
    bool equals(solver::Observation const &otherObs) const override;
    std::size_t hash() const override;
    void print(std::ostream &os) const override;

    long getBinNumber() const override;

    /** Returns true iff this observation is empty. */
    bool isEmpty() const;
    /** Returns true iff this observation is good. */
    bool isGood() const;

  private:
    /** True iff this observation is empty. */
    bool isEmpty_;
    /** True iff this observation is good. */
    bool isGood_;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_OBSERVATION_HPP_ */
