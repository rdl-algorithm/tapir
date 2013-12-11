#ifndef ACTIONNODE_HPP
#define ACTIONNODE_HPP

#include <memory>                       // for unique_ptr
#include <queue>                        // for queue
#include <utility>                      // for pair
#include <vector>                       // for vector

#include "Action.hpp"                   // for Action
#include "Observation.hpp"              // for Observation

class BeliefNode;
class ObservationEdge;

//#include "ObservationEdge.hpp"          // for ObservationEdge


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

    /** Enqueues all belief children onto the given queue */
    void enqueueChildren(std::queue<BeliefNode *> &queue);

  private:
    /** Constructs an action node without an action! */
    ActionNode();
    /** Adds the given ObservationEdge as a child, and takes possession of
     * it.
     */
    void addChild(std::unique_ptr<ObservationEdge> edge);

    /** The action for this node. */
    Action action;
    /** The number of particles for this node. */
    unsigned long nParticles;
    /** The total q-value of this node. */
    double totalQValue;
    /** The mean q-value of this node. */
    double meanQValue;

    /** The vector of this node's children. */
    std::vector<std::unique_ptr<ObservationEdge>> obsChildren;
};

#endif /* ACTIONNODE_HPP */
