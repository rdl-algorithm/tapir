#ifndef SOLVER_ACTIONNODE_HPP_
#define SOLVER_ACTIONNODE_HPP_

#include <memory>                       // for unique_ptr
#include <utility>                      // for pair
#include <vector>                       // for vector

#include "geometry/Action.hpp"                   // for Action
#include "geometry/Observation.hpp"              // for Observation
#include "mappings/ObservationMapping.hpp"
#include "global.hpp"

namespace solver {
class BeliefNode;

class ActionNode {
  public:
    friend class TextSerializer;

    /** Constructs an action node without an action!! */
    ActionNode();
    /** Creates an empty action node with the given action */
    ActionNode(std::unique_ptr<ObservationMapping> mapping,
            Action const *action);

    // Default destructor; copying and moving disallowed!
    ~ActionNode();
    _NO_COPY_OR_MOVE(ActionNode);

    /** Updates the q-value, as would occur on adding the given amount to the
     * total (negative for a decrease).
     */
    void updateQValue(double increase);
    /** Updates the q-value, as would occur on adding the given amount to the
     * total, and changing the number of particles by deltaNParticles (e.g.
     * +1 to count a new particle; -1 to remove a particle.
     */
    void updateQValue(double increase, long deltaNParticles);

    /** Returns the number of particles counted towards the q-value. */
    long getNParticles() const;
    /** Returns the total q-value for this node. */
    double getTotalQValue () const;
    /** Returns the mean q-value for this node. */
    double getMeanQValue () const;

    /** Returns the child corresponding to the given observation, based on
     * sufficient proximity.
     */
    BeliefNode *getChild(Observation const &obs) const;

    /** Adds a child with the given observation, creating a new belief node if
     * necessary.
     */
    std::pair<BeliefNode *, bool> createOrGetChild(Observation const &obs);

    /** Returns the action used within this ActionNode. */
    Action const *getAction() const;

  private:
    /** The action for this node. */
    std::unique_ptr<Action> action_;
    /** The number of particles being counted towards the q-value for this
     * node.
     */
    long nParticles_;
    /** The total q-value of this node. */
    double totalQValue_;
    /** The mean q-value of this node. */
    double meanQValue_;

    std::unique_ptr<ObservationMapping> obsMap_;
};
} /* namespace solver */

#endif /* SOLVER_ACTIONNODE_HPP_ */
