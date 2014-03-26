#ifndef SOLVER_OBSERVATIONMAPPING_HPP_
#define SOLVER_OBSERVATIONMAPPING_HPP_

#include <memory>                       // for unique_ptr

#include "solver/abstract-problem/Observation.hpp"              // for Observation

namespace solver {
class BeliefNode;

class ObservationMapping {
  public:
    /** Creates an empty observation map */
    ObservationMapping() = default;

    // Default destructor; copying and moving disallowed!
    virtual ~ObservationMapping() = default;
    ObservationMapping(ObservationMapping const &) = delete;
    ObservationMapping(ObservationMapping &&) = delete;
    ObservationMapping &operator=(ObservationMapping const &) = delete;
    ObservationMapping &operator=(ObservationMapping &&) = delete;

    /* -------------- Creation and retrieval of nodes. ---------------- */
    /** Retrieves the belief node (if any) corresponding to this observation */
    virtual BeliefNode *getBelief(Observation const &obs) const = 0;
    /** Creates a new belief node for the given observation */
    virtual BeliefNode *createBelief(Observation const &obs) = 0;

    /* ------------------- General getters --------------------- */
    /** Returns the number of child nodes associated with this mapping. */
    virtual long getNChildren() const = 0;

    /* --------------- Methods for accessing visit counts. ----------------- */
    /** Updates the visit count for the given observation. */
    virtual void updateVisitCount(Observation const &obs, long deltaNVisits) = 0;
    /** Returns the visit count for the given observation. */
    virtual long getVisitCount(Observation const &obs) const = 0;
    /** Returns the total visit count among all observations. */
    virtual long getTotalVisitCount() const = 0;
};
} /* namespace solver */

#endif /* SOLVER_OBSERVATIONMAPPING_HPP_ */
