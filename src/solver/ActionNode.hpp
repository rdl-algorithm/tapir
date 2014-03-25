#ifndef SOLVER_ACTIONNODE_HPP_
#define SOLVER_ACTIONNODE_HPP_

#include <memory>                       // for unique_ptr
#include <utility>                      // for pair
#include <vector>                       // for vector

#include "abstract-problem/Action.hpp"                   // for Action
#include "abstract-problem/Observation.hpp"              // for Observation
#include "mappings/ObservationMapping.hpp"
#include "global.hpp"

namespace solver {
class BeliefNode;

class ActionNode {
  public:
    friend class TextSerializer;

    /** Constructs an action node without an observation mapping! */
    ActionNode();
    /** Creates an empty action node with the given observation mapping.*/
    ActionNode(std::unique_ptr<ObservationMapping> mapping);

    // Default destructor; copying and moving disallowed!
    ~ActionNode();
    _NO_COPY_OR_MOVE(ActionNode);

    /* --------------------- Tree-related getters ----------------------- */
    /** Returns the observation mapping for this node. */
    ObservationMapping *getMapping();
    /** Returns the child corresponding to the given observation, based on
     * sufficient proximity.
     */
    BeliefNode *getChild(Observation const &obs) const;
    /** Adds a child with the given observation, creating a new belief node if
     * necessary.
     */
    std::pair<BeliefNode *, bool> createOrGetChild(Observation const &obs);

  private:
    std::unique_ptr<ObservationMapping> observationMap_;
};
} /* namespace solver */

#endif /* SOLVER_ACTIONNODE_HPP_ */
