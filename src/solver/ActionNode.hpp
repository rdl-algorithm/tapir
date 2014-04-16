#ifndef SOLVER_ACTIONNODE_HPP_
#define SOLVER_ACTIONNODE_HPP_

#include <memory>                       // for unique_ptr
#include <utility>                      // for pair
#include <vector>                       // for vector

#include "global.hpp"

#include "abstract-problem/Action.hpp"                   // for Action
#include "abstract-problem/Observation.hpp"              // for Observation

#include "mappings/ObservationMapping.hpp"

namespace solver {
class ActionMappingEntry;
class BeliefNode;
class ObservationMapping;

class ActionNode {
  public:
    friend class TextSerializer;

    /** Constructs an action node without an observation mapping! */
    ActionNode();
    /** Constructs an action node with the given entry as its parent. */
    ActionNode(ActionMappingEntry *parentEntry);

    // Default destructor; copying and moving disallowed!
    ~ActionNode();
    _NO_COPY_OR_MOVE(ActionNode);

    /* -------------------- Tree-related setters  ---------------------- */
    /** Sets the mapping for this node. */
    void setMapping(std::unique_ptr<ObservationMapping> mapping);
    /** Sets the parent entry for this node. */
    void setParentEntry(ActionMappingEntry *entry);

    /* -------------------- Tree-related getters  ---------------------- */
    /** Returns the observation mapping for this node. */
    ObservationMapping *getMapping() const;
    /** Returns the parent entry for this node. */
    ActionMappingEntry *getParentEntry() const;
    /** Returns the child corresponding to the given observation, based on
     * sufficient proximity.
     */
    BeliefNode *getChild(Observation const &obs) const;

    /* -------------------- Tree-related methods  ---------------------- */
    /** Adds a child with the given observation, creating a new belief node if
     * necessary.
     */
    std::pair<BeliefNode *, bool> createOrGetChild(
            Observation const &obs);

  private:
    ActionMappingEntry *parentEntry_;
    std::unique_ptr<ObservationMapping> observationMap_;
};
} /* namespace solver */

#endif /* SOLVER_ACTIONNODE_HPP_ */
