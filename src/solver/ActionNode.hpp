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

    /** Constructs an action node without an action!! */
    ActionNode();
    /** Creates an empty action node with the given action */
    ActionNode(Action const &action);

    // Default destructor; copying and moving disallowed!
    ~ActionNode();
    ActionNode(ActionNode const &) = delete;
    ActionNode(ActionNode &&) = delete;
    ActionNode &operator=(ActionNode const &) = delete;
    ActionNode &operator=(ActionNode &&) = delete;

    /** Updates the q-value, as would occur on adding the given amount to the
     * total (negative for a decrease).
     */
    void updateQValue(double increase);
    /** Updates the q-value, as would occur on adding the given amount to the
     * total, and changing the number of particles by deltaNParticles (e.g.
     * +1 to count a new particle; -1 to remove a particle.
     */
    void updateQValue(double increase, long deltaNParticles);

    /** Adds a new ObservationEdge with the given observation, creating
     * a new belief node if necessary.
     */
    std::pair<BeliefNode *, bool> addChild(Observation const &obs);

    /** Returns the child corresponding to the given observation, based on
     * sufficient proximity.
     */
    BeliefNode *getBeliefChild(Observation const &obs);

  private:
    /** The action for this node. */
    Action action_;
    /** The number of particles being counted towards the q-value for this
     * node.
     */
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
