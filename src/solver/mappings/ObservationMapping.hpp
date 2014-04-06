#ifndef SOLVER_OBSERVATIONMAPPING_HPP_
#define SOLVER_OBSERVATIONMAPPING_HPP_

#include <memory>                       // for unique_ptr

#include "global.hpp"

#include "solver/abstract-problem/Observation.hpp"              // for Observation

namespace solver {
class ActionNode;
class BeliefNode;
class ObservationMappingEntry;

class ObservationMapping {
  public:
    ObservationMapping() = default;
    virtual ~ObservationMapping() = default;
    _NO_COPY_OR_MOVE(ObservationMapping);

    /* -------------- Association with an action node ---------------- */
    /* Associates this mapping with the given action node. */
    virtual void setOwner(ActionNode *owner) = 0;
    /** Returns the action node that owns this mapping. */
    virtual ActionNode *getOwner() const = 0;

    /* -------------- Creation and retrieval of nodes. ---------------- */
    /** Retrieves the belief node (if any) corresponding to this observation */
    virtual BeliefNode *getBelief(Observation const &obs) const = 0;
    /** Creates a new belief node for the given observation */
    virtual BeliefNode *createBelief(Observation const &obs) = 0;

    /* -------------- Retrieval of mapping entries. ---------------- */
    /** Returns the number of child nodes associated with this mapping. */
    virtual long getNChildren() const = 0;
    /** Returns the mapping entry associated with the given observation. */
    virtual ObservationMappingEntry const *getEntry(Observation const &obs) const = 0;

    /* ------------- Methods for accessing visit counts. --------------- */
    /** Updates the visit count for the given observation. */
    virtual void updateVisitCount(Observation const &obs, long deltaNVisits) = 0;
    /** Returns the visit count for the given observation. */
    virtual long getVisitCount(Observation const &obs) const = 0;
    /** Returns the total visit count among all observations. */
    virtual long getTotalVisitCount() const = 0;
};

class ObservationMappingEntry {
public:
    ObservationMappingEntry() = default;
    virtual ~ObservationMappingEntry() = default;
    _NO_COPY_OR_MOVE(ObservationMappingEntry);

    /** Returns the mapping this entry belongs to. */
    virtual ObservationMapping *getMapping() const = 0;
    /** Returns the observation for this entry. */
    virtual std::unique_ptr<Observation> getObservation() const = 0;
    /** Returns the belief node for this entry. */
    virtual BeliefNode *getBeliefNode() const = 0;
    /** Returns the visit count for this entry. */
    virtual long getVisitCount() const = 0;
};
} /* namespace solver */

#endif /* SOLVER_OBSERVATIONMAPPING_HPP_ */
