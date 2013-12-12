#ifndef SOLVER_ACTIONNODE_HPP_
#define SOLVER_ACTIONNODE_HPP_

#include <memory>                       // for unique_ptr
#include <utility>                      // for pair
#include <vector>                       // for vector

#include "Action.hpp"                   // for Action
#include "Observation.hpp"              // for Observation

namespace solver {
class BeliefNode;
class ObservationEdge;

class ActionNode {
  public:
    friend class BeliefNode;
    friend class TextSerializer;

    /** Creates an empty action node with the given action */
    ActionNode(Action const &action);

    /** Default destructor. */
    ~ActionNode();
    /* Copying and moving is disallowed. */
    ActionNode(ActionNode const &) = delete;
    ActionNode(ActionNode &&) = delete;
    ActionNode &operator=(ActionNode const &) = delete;
    ActionNode &operator=(ActionNode &&) = delete;

    /** Updates the q-value, as would occur on adding the given amount to the
     * total.
     */
    void updateQValue(double increase);
    /** Updates the q-value, as would occur on replacing the old value with
     * the new value (and reducing the # of particles by 1 if reduceParticles
     * is true).
     */
    void updateQValue(double oldValue, double newValue, bool reduceParticles);

    /** Adds a new ObservationEdge with the given observation, creating
     * a new belief node if necessary.
     */
    std::pair<BeliefNode *, bool> addChild(Observation const &obs);

    /** Returns the child corresponding to the given observation, based on
     * sufficient proximity.
     */
    BeliefNode *getBeliefChild(Observation const &obs);

  private:
    /** Constructs an action node without an action! */
    ActionNode();

    /** The action for this node. */
    Action action_;
    /** The number of particles for this node. */
    unsigned long nParticles_;
    /** The total q-value of this node. */
    double totalQValue_;
    /** The mean q-value of this node. */
    double meanQValue_;

    /** The vector of this node's children. */
    std::vector<std::unique_ptr<ObservationEdge>> obsChildren;
};
} /* namespace solver */

#endif /* SOLVER_ACTIONNODE_HPP_ */
