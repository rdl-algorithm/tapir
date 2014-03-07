#ifndef ROCKSAMPLE_OBSERVATION_HPP_
#define ROCKSAMPLE_OBSERVATION_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "solver/abstract-problem/EnumeratedPoint.hpp"
#include "solver/abstract-problem/Observation.hpp"

#include "global.hpp"                     // for RandomGenerator

namespace rocksample {
class RockSampleObservation : public solver::EnumeratedPoint {
    friend class RockSampleTextSerializer;
  public:
    RockSampleObservation();
    RockSampleObservation(bool _isGood);
    RockSampleObservation(bool _isEmpty, bool _isGood);
    RockSampleObservation(long code);

    virtual ~RockSampleObservation() = default;
    _NO_COPY_OR_MOVE(RockSampleObservation);

    std::unique_ptr<solver::Observation> copy() const override;
    double distanceTo(solver::Observation const &otherObs) const override;
    bool equals(solver::Observation const &otherObs) const override;
    std::size_t hash() const override;
    void print(std::ostream &os) const override;

    long getCode() const override;

    bool isEmpty() const;
    bool isGood() const;

  private:
    bool isEmpty_;
    bool isGood_;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_OBSERVATION_HPP_ */
