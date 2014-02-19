#ifndef ROCKSAMPLE_OBSERVATION_HPP_
#define ROCKSAMPLE_OBSERVATION_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/Observation.hpp"             // for State

namespace rocksample {
class RockSampleObservation : public solver::Observation {
    friend class RockSampleTextSerializer;
  public:
    RockSampleObservation();
    RockSampleObservation(bool _isGood);
    RockSampleObservation(bool _isEmpty, bool _isGood);

    virtual ~RockSampleObservation() = default;
    RockSampleObservation(RockSampleObservation const &other) = delete;
    RockSampleObservation(RockSampleObservation &&) = delete;
    virtual RockSampleObservation &operator=(RockSampleObservation const &) = delete;
    virtual RockSampleObservation &operator=(RockSampleObservation &&) = delete;

    std::unique_ptr<solver::Observation> copy() const;

    double distanceTo(solver::Observation const &otherObs) const;
    bool equals(solver::Observation const &otherObs) const;
    std::size_t hash() const;
    void print(std::ostream &os) const;

    bool isEmpty() const;
    bool isGood() const;

  private:
    bool isEmpty_;
    bool isGood_;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_OBSERVATION_HPP_ */
