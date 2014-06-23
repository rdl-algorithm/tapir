#ifndef SOLVER_OBSERVATIONMAPPING_HPP_
#define SOLVER_OBSERVATIONMAPPING_HPP_

#include <memory>                       // for unique_ptr

#include "global.hpp"

#include "solver/abstract-problem/Observation.hpp"              // for Observation

#include "ObservationMappingEntry.hpp"

namespace solver {
class ActionNode;
class BeliefNode;

class ObservationMapping {
public:
    ObservationMapping(ActionNode *owner) :
        owner_(owner) {
    }
    virtual ~ObservationMapping() = default;
    _NO_COPY_OR_MOVE(ObservationMapping);

    /* -------------- Association with an action node ---------------- */
    /** Returns the action node that owns this mapping. */
    ActionNode *getOwner() const {
        return owner_;
    }

    /* -------------- Creation and retrieval of nodes. ---------------- */
    /** Retrieves the belief node (if any) corresponding to this observation */
    virtual BeliefNode *getBelief(Observation const &obs) const = 0;
    /** Creates a new belief node for the given observation */
    virtual BeliefNode *createBelief(Observation const &obs) = 0;

    /* -------------- Retrieval of mapping entries. ---------------- */
    /** Returns the number of child nodes associated with this mapping. */
    virtual long getNChildren() const = 0;
    /** Returns the mapping entry associated with the given observation. */
    virtual ObservationMappingEntry *getEntry(Observation const &obs) = 0;
    /** Returns the mapping entry associated with the given observation. */
    virtual ObservationMappingEntry const *getEntry(Observation const &obs) const = 0;
    /** Returns a vector of all of the entries in this mapping. */
    virtual std::vector<ObservationMappingEntry const *> getAllEntries() const = 0;

    /* ------------- Methods for accessing visit counts. --------------- */
    /** Updates the visit count for the given observation. */
    virtual void updateVisitCount(Observation const &obs, long deltaNVisits) {
        getEntry(obs)->updateVisitCount(deltaNVisits);
    }
    /** Returns the visit count for the given observation. */
    virtual long getVisitCount(Observation const &obs) const {
        return getEntry(obs)->getVisitCount();
    }
    /** Returns the total visit count among all observations. */
    virtual long getTotalVisitCount() const = 0;
private:
    ActionNode *owner_;
};
} /* namespace solver */

#endif /* SOLVER_OBSERVATIONMAPPING_HPP_ */
