#ifndef SOLVER_OBSERVATIONMAPPINGENTRY_HPP_
#define SOLVER_OBSERVATIONMAPPINGENTRY_HPP_

#include <memory>                       // for unique_ptr

#include "global.hpp"

#include "solver/abstract-problem/Observation.hpp"              // for Observation

namespace solver {
class ActionNode;
class BeliefNode;
class ObservationMapping;

class ObservationMappingEntry {
public:
    ObservationMappingEntry() = default;
    virtual ~ObservationMappingEntry() = default;

    /** Returns the mapping this entry belongs to. */
    virtual ObservationMapping *getMapping() const = 0;
    /** Returns the observation for this entry. */
    virtual std::unique_ptr<Observation> getObservation() const = 0;
    /** Returns the belief node for this entry. */
    virtual BeliefNode *getBeliefNode() const = 0;
    /** Returns the visit count for this entry. */
    virtual long getVisitCount() const = 0;

    /** Updates the visit count for this observation. */
    virtual void updateVisitCount(long deltaNVisits) = 0;
};
} /* namespace solver */

#endif /* SOLVER_OBSERVATIONMAPPINGENTRY_HPP_ */
