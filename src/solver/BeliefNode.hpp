#ifndef SOLVER_BELIEFNODE_HPP_
#define SOLVER_BELIEFNODE_HPP_

#include <map>                          // for map, map<>::value_compare
#include <memory>                       // for unique_ptr
#include <set>
#include <utility>                      // for pair

#include "global.hpp"                     // for RandomGenerator

#include "abstract-problem/Action.hpp"                   // for Action
#include "abstract-problem/State.hpp"
#include "abstract-problem/Observation.hpp"              // for Observation
#include "RandomAccessSet.hpp"

namespace solver {
class ActionMapping;
class ActionNode;
class HistoryEntry;

class BeliefNode {
  public:
    friend class Solver;
    friend class TextSerializer;

    /** Constructs a new belief node with no ID (-1). */
    BeliefNode();
    /** Constructs a new belief node with the given ID and no action mapping. */
    BeliefNode(long id);

    // Default destructor; copying and moving disallowed!
    ~BeliefNode();
    _NO_COPY_OR_MOVE(BeliefNode);

    /* -------------- Particle management / sampling ---------------- */
    /** Adds the given history entry to this belief node. */
    void addParticle(HistoryEntry *newHistEntry);
    /** Removes the given history entry from this belief node. */
    void removeParticle(HistoryEntry *histEntry);
    /** Samples a particle from this node. */
    HistoryEntry *sampleAParticle(RandomGenerator *randGen) const;

    /* ---------------- Useful calculations ------------------ */
    /** Calculates the distance between this belief node and another by
     * calculating the average pairwise distance between the individual
     * particles.
     */
    double distL1Independent(BeliefNode *b) const;

    /* -------------------- Simple setters ---------------------- */
    /** Sets the id of this node. */
    void setId(long id);

    /* -------------------- Simple getters ---------------------- */
    /** Returns the id of this node. */
    long getId() const;
    /** Chooses the action with the best expected value. */
    std::unique_ptr<Action> getBestAction() const;
    /** Returns the best q-value */
    double getQValue() const;
    /** Returns the number of particles in this node. */
    long getNumberOfParticles() const;
    /** Returns the number of sequences starting from this node. */
    long getNumberOfStartingSequences() const;
    /** Returns a vector containing all of the states contained in node. */
    std::vector<State const *> getStates() const;
    /** Returns the time at which the last change occurred. */
    double getTimeOfLastChange() const;

    /* -------------------- Tree-related setters  ---------------------- */
    /** Sets the mapping for this node. */
    void setMapping(std::unique_ptr<ActionMapping> mapping);

    /* -------------------- Tree-related getters  ---------------------- */
    /** Returns the action mapping for this node. */
    ActionMapping *getMapping();
    /** Returns the belief node child corresponding to the given action and
     * observation
     */
    BeliefNode *getChild(Action const &action, Observation const &obs) const;

    /* -------------------- Tree-related methods  ---------------------- */
    /** Adds a child for the given action and observation;
     * returns the child node, and a boolean which is true iff a new node was
     * actually created.
     */
    std::pair<BeliefNode *, bool> createOrGetChild(Action const &action,
            Observation const &obs);

private:
    /** The ID of this node. */
    long id_;

    /** The set of particles belonging to this node. */
    abt::RandomAccessSet<HistoryEntry *> particles_;
    /** The number of sequences that start at this node. */
    long nStartingSequences_;
    /** The time at which the last particle modification occurred. */
    double tLastChange_;

    /** A mapping of actions to action children for this node. */
    std::unique_ptr<ActionMapping> actionMap_;
};
} /* namespace solver */

#endif /* SOLVER_BELIEFNODE_HPP_ */
