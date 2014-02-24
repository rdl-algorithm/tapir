#ifndef SOLVER_OBSERVATIONMAPPING_HPP_
#define SOLVER_OBSERVATIONMAPPING_HPP_

#include <memory>                       // for unique_ptr

#include "solver/geometry/Observation.hpp"              // for Observation

namespace solver {
class BeliefNode;

class ObservationMapping {
  public:
    friend class TextSerializer;

    /** Creates an empty observation map */
    ObservationMapping() = default;

    // Default destructor; copying and moving disallowed!
    virtual ~ObservationMapping() = default;
    ObservationMapping(ObservationMapping const &) = delete;
    ObservationMapping(ObservationMapping &&) = delete;
    ObservationMapping &operator=(ObservationMapping const &) = delete;
    ObservationMapping &operator=(ObservationMapping &&) = delete;

    /** Retrieves the belief node (if any) corresponding to this observation */
    virtual BeliefNode *getBelief(Observation const &obs) const = 0;

    /** Creates a new belief node for the given observation */
    virtual BeliefNode *createBelief(Observation const &obs) = 0;
};
} /* namespace solver */

#endif /* SOLVER_OBSERVATIONMAPPING_HPP_ */
