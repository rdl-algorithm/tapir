#ifndef SOLVER_BELIEFNODE_HPP_
#define SOLVER_BELIEFNODE_HPP_

#include <ctime>                        // for clock_t

#include <map>                          // for map, map<>::value_compare
#include <memory>                       // for unique_ptr
#include <set>
#include <utility>                      // for pair

#include "defs.hpp"                     // for RandomGenerator

#include "Action.hpp"                   // for Action
#include "Observation.hpp"              // for Observation
#include "ParticleSet.hpp"

namespace solver {
class ActionNode;
class HistoryEntry;

class BeliefNode {
  public:
    friend class Solver;
    friend class TextSerializer;

    /** Constructs a new belief node with an auto-generated ID. */
    BeliefNode();
    /** Constructs a new belief node with the given ID. */
    BeliefNode(long id);
    /** Default destructor. */
    ~BeliefNode();

    /* Copying and moving is disallowed. */
    BeliefNode(BeliefNode const &) = delete;
    BeliefNode(BeliefNode &&) = delete;
    BeliefNode &operator=(BeliefNode const &) = delete;
    BeliefNode &operator=(BeliefNode &&) = delete;

    /** Chooses a next action with the UCB algorithm. */
    Action getUcbAction(double ucbExploreCoefficient);
    /** Chooses the action with the best expected value */
    Action getBestAction();
    /** Updates the calculation of which action is optimal. */
    void updateBestValue();
    /** Returns the best q-value */
    double getBestMeanQValue();

    /** Adds the given history entry to this belief node. */
    void addParticle(HistoryEntry *newHistEntry);
    /** Removes the given history entry from this belief node. */
    void removeParticle(HistoryEntry *histEntry);

    /** Adds a child for the given action and observation;
     * returns the child node, and a boolean representing
     */
    std::pair<BeliefNode *, bool> addChild(Action const &action,
            Observation const &obs);
    /** Samples a particle from this node. */
    HistoryEntry *sampleAParticle(RandomGenerator *randGen);

    /** Updates the q-value for the given action, with the given increase. */
    void updateQValue(Action &action, double increase);
    /** Updates the q-value for the given action, as would occur if replacing
     * the old value with the new value, and reducing the number of particles
     * for the action node if reduceParticles is true.
     */
    void updateQValue(Action &action, double oldValue, double newValue,
            bool reduceParticles);

    /** Calculates the distance between this belief node and another by
     * calculating the average pairwise distance between the individual
     * particles.
     */
    double distL1Independent(BeliefNode *b);

    /** Returns the belief node child corresponding to the given action and
     * observation
     */
    BeliefNode *getChild(Action const &action, Observation const &obs);

    /** Returns the next action to attempt. */
    Action getNextActionToTry();

    /** Returns the ID of this node. */
    long getId() {
        return id_;
    }
    /** Returns the number of particles in this node. */
    long getNParticles() {
        return particles_.size();
    }
    /** Returns the current number of action children of this node. */
    unsigned long getNActChildren() {
        return actChildren_.size();
    }

  private:
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
