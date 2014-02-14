#ifndef SOLVER_BELIEFNODE_HPP_
#define SOLVER_BELIEFNODE_HPP_

#include <ctime>                        // for clock_t

#include <map>                          // for map, map<>::value_compare
#include <memory>                       // for unique_ptr
#include <set>
#include <utility>                      // for pair

#include "global.hpp"                     // for RandomGenerator

#include "Action.hpp"                   // for Action
#include "Observation.hpp"              // for Observation
#include "ParticleSet.hpp"

namespace solver {
class ActionNode;
class HistoryEntry;

class BeliefNode {
  public:
    friend class BeliefTree;
    friend class Solver;
    friend class TextSerializer;

    /** Constructs a new belief node with an auto-generated ID. */
    BeliefNode();
    /** Constructs a new belief node with the given ID. */
    BeliefNode(long id);

    // Default destructor; copying and moving disallowed!
    ~BeliefNode();
    BeliefNode(BeliefNode const &) = delete;
    BeliefNode(BeliefNode &&) = delete;
    BeliefNode &operator=(BeliefNode const &) = delete;
    BeliefNode &operator=(BeliefNode &&) = delete;

    /** Chooses a next action with the UCB algorithm. */
    Action getUcbAction(double ucbExploreCoefficient);
    /** Updates the calculation of which action is optimal. */
    void updateBestValue();
    /** Chooses the action with the best expected value */
    Action getBestAction() const;
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
    double distL1Independent(BeliefNode *b);

    /** Returns the belief node child corresponding to the given action and
     * observation
     */
    BeliefNode *getChild(Action const &action, Observation const &obs) const;

    /** Returns the next action to attempt. */
    Action getNextActionToTry();

    /** Returns the ID of this node. */
    long getId() const;

    /** Returns the number of particles in this node. */
    long getNParticles() const;

    /** Returns the current number of action children of this node. */
    long getNActChildren() const;

  private:
    /** Adds a child for the given action and observation;
     * returns the child node, and a boolean representing
     */
    std::pair<BeliefNode *, bool> createOrGetChild(Action const &action,
            Observation const &obs);

    /** The ID for the next belief node. */
    static long currId;
    /** The startup time */
    static std::clock_t startTime;

    /** The ID of this node. */
    long id_;
    /** The next untried action to explore. */
    Action nextActionToTry_;
    /** The best mean q-value of any action child. */
    double bestMeanQValue_;
    /** The action corresponding to the highest expected value. */
    Action bestAction_;

    /** The time at which the last particle was added. */
    double tLastAddedParticle_;
    /** The time at which the last belief node comparison was done. */
    double tNNComp_;
    /** A previously found near neighbor for this belief node. */
    BeliefNode *nnBel_;

    /** The set of particles belonging to this node. */
    ParticleSet particles_;

    typedef std::map<Action, std::unique_ptr<ActionNode>> ActionMap;
    /** A mapping of actions to action children for this node. */
    ActionMap actChildren_;
};
} /* namespace solver */

#endif /* SOLVER_BELIEFNODE_HPP_ */
