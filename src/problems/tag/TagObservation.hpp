#ifndef TAG_OBSERVATION_HPP_
#define TAG_OBSERVATION_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "problems/shared/GridPosition.hpp"
#include "solver/abstract-problem/DiscretizedPoint.hpp"
#include "solver/abstract-problem/Observation.hpp"

#include "global.hpp"                     // for RandomGenerator

namespace tag {
class TagModel;

class TagObservation : public solver::Point {
    friend class TagTextSerializer;
  public:
    TagObservation(GridPosition myPosition, bool seesOpponent = false);

    virtual ~TagObservation() = default;
    _NO_COPY_OR_MOVE(TagObservation);

    std::unique_ptr<solver::Observation> copy() const override;
    double distanceTo(solver::Observation const &otherObs) const override;
    bool equals(solver::Observation const &otherObs) const override;
    std::size_t hash() const override;
    void print(std::ostream &os) const override;

    GridPosition getPosition() const;
    bool seesOpponent() const;

  private:
    GridPosition position_;
    bool seesOpponent_;
};
} /* namespace tag */
#endif /* TAG_OBSERVATION_HPP_ */
