#ifndef SOLVER_BELIEFNODE_HPP_
#define SOLVER_BELIEFNODE_HPP_

#include <ctime>                        // for clock_t

#include <map>                          // for map, map<>::value_compare
#include <memory>                       // for unique_ptr
#include <set>
#include <utility>                      // for pair

#include "global.hpp"                     // for RandomGenerator

#include "geometry/Action.hpp"                   // for Action
#include "geometry/State.hpp"
#include "geometry/Observation.hpp"              // for Observation
#include "RandomAccessSet.hpp"

namespace solver {
class ActionMapping;
class ActionNode;
class HistoryEntry;

class BeliefNode {
  public:
    friend class Solver;
    friend class TextSerializer;

    /** Constructs a new belief node with the given ID and no action mapping. */
    BeliefNode(long id);
    /** Constructs a new belief node with no ID (-1). */
    BeliefNode(std::unique_ptr<ActionMapping> actionMap);
    /** Constructs a new belief node with the given ID. */
    BeliefNode(std::unique_ptr<ActionMapping> actionMap, long id);

    // Default destructor; copying and moving disallowed!
    ~BeliefNode();
    _NO_COPY_OR_MOVE(BeliefNode);

    /** Returns true iff there is another action that needs to be explored. */
    bool hasActionToTry() const;
    /** Returns the next action to attempt, if any. */
    std::unique_ptr<Action> getNextActionToTry() const;

    /** Chooses the next action to search, from a multi-armed bandit problem
     * (probably using a UCB algorithm). */
    std::unique_ptr<Action> getSearchAction(double ucbExploreCoefficient);

    /** Updates the calculation of which action is optimal. */
    void updateBestValue();
    /** Chooses the action with the best expected value */
    std::unique_ptr<Action> getBestAction() const;
    /** Returns the best q-value */
    double getBestMeanQValue() const;


    /** Adds the given history entry to this belief node. */
    void addParticle(HistoryEntry *newHistEntry);
    /** Removes the given history entry from this belief node. */
    void removeParticle(HistoryEntry *histEntry);
    /** Samples a particle from this node. */
    HistoryEntry *sampleAParticle(RandomGenerator *randGen) const;


    /** Updates the q-value for the given action, with the given increase in
     * the total q-value (negative values for a decrease).
     */
    void updateQValue(Action const &action, double increase);
    /** Updates the q-value for the given action, with the given increase in
     * the total q-value (negative values for a decrease), and the given
     * change in the recorded number of particles (0 for no change, -1 for a
     * reduction, and +1 for an increase.
     */
    void updateQValue(Action const &action, double increase,
            long deltaNParticles);


    /** Calculates the distance between this belief node and another by
     * calculating the average pairwise distance between the individual
     * particles.
     */
    double distL1Independent(BeliefNode *b) const;


    /** Returns the id of this node. */
    long getId() const;
    /** Sets the id of this node. */
    void setId(long id);
    /** Returns the number of particles in this node. */
    long getNParticles() const;
    /** Returns an array containing all of the states within this node. */
    std::vector<State const *> getStates() const;
    /** Returns the current number of action children of this node. */
    long getNActChildren() const;

    /** Returns the action mapping for this node. */
    ActionMapping *getMapping();
    /** Returns the belief node child corresponding to the given action and
     * observation
     */
    BeliefNode *getChild(Action const &action, Observation const &obs) const;
    /** Adds a child for the given action and observation;
     * returns the child node, and a boolean which is true iff a new node was
     * actually created.
     */
    std::pair<BeliefNode *, bool> createOrGetChild(Action const &action,
            Observation const &obs);

private:
    /** The ID of this node. */
    long id_;

    /** The time at which the last particle was added. */
    double tLastAddedParticle_;
    /** The time at which the last belief node comparison was done. */
    double tNNComp_;
    /** A previously found near neighbor for this belief node. */
    BeliefNode *nnBel_;

    /** The set of particles belonging to this node. */
    abt::RandomAccessSet<HistoryEntry *> particles_;

    /** A mapping of actions to action children for this node. */
    std::unique_ptr<ActionMapping> actionMap_;
};
} /* namespace solver */

#endif /* SOLVER_BELIEFNODE_HPP_ */
