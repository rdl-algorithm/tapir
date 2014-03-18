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

    /* -------------------- Q-value update methods ---------------------- */
    /** Changes the q-value directly. */
    void changeTotalQValue(double deltaQ, long deltaNParticles);
    /** Updates the q-value of the child belief node with the given
     * observation, and updates this action node to reflect the effects of
     * the change in the child's q-value.
     *
     * The discount factor is applied to the child belief's value, whereas
     * deltaNSequences is the # of sequences added or removed. Note that, due
     * to the nature of the update mechanism, adding new sequences should
     * happen *before* this update, but removals should happen *after*.
     */
    void updateChildQValue(Observation const &observation,
            double discountFactor, long deltaNSequences);
    /** Recalculates the mean q-value - this method should be used after the
     * total or the particle count changes.
     */
    void recalculateQValue();

    /* ------------------------ Simple getters -------------------------- */
    /** Returns the number of particles counted towards the q-value. */
    long getNParticles() const;
    /** Returns the total q-value for this node. */
    double getTotalQValue() const;
    /** Returns the q-value for this node. */
    double getQValue() const;

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
    /** The number of particles counted towards the q-value for this node. */
    long nParticles_;
    /** The total q-value of this node. */
    double totalQValue_;
    /** The mean q-value of this node. */
    double meanQValue_;

    std::unique_ptr<ObservationMapping> observationMap_;
};
} /* namespace solver */

#endif /* SOLVER_ACTIONNODE_HPP_ */
