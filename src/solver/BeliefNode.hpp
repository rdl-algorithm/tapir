#ifndef SOLVER_BELIEFNODE_HPP_
#define SOLVER_BELIEFNODE_HPP_

#include <map>                          // for map, map<>::value_compare
#include <memory>                       // for unique_ptr
#include <set>
#include <utility>                      // for pair

#include "global.hpp"                     // for RandomGenerator
#include "RandomAccessSet.hpp"

#include "abstract-problem/Action.hpp"                   // for Action
#include "abstract-problem/State.hpp"
#include "abstract-problem/Observation.hpp"              // for Observation

#include "belief-q-estimators/estimation.hpp"

#include "mappings/actions/ActionMapping.hpp"

#include "search/HistoricalData.hpp"

namespace solver {
class ActionMapping;
class ActionNode;
class HistoricalData;
class HistoryEntry;
class ObservationMappingEntry;
class Solver;

class BeliefNode {
public:
    friend class ActionNode;
    friend class BeliefTree;
    friend class HistoryEntry;
    friend class Solver;
    friend class TextSerializer;

    /** Constructs a new belief node with no ID, and no parent entry. */
    BeliefNode();
    /** Constructs a new belief node with no ID (-1),
     * and with the given mapping entry as its parent.
     */
    BeliefNode(ObservationMappingEntry *parentEntry);
    /** Constructs a new belief node with the given ID,
     * and with the given mapping entry as its parent.
     */
    BeliefNode(long id, ObservationMappingEntry *parentEntry);

    // Default destructor; copying and moving disallowed!
    ~BeliefNode();
    _NO_COPY_OR_MOVE(BeliefNode);

    /* ---------------- Useful calculations ------------------ */
    /** Calculates the distance between this belief node and another by
     * calculating the average pairwise distance between the individual
     * particles.
     */
    double distL1Independent(BeliefNode *b) const;

    /* -------------------- Simple getters ---------------------- */
    /** Returns the id of this node. */
    long getId() const;
    /** Returns the depth of this node in the tree (0 = root). */
    long getDepth() const;
    /** Returns the number of particles in this node. */
    long getNumberOfParticles() const;
    /** Returns the number of sequences starting from this node. */
    long getNumberOfStartingSequences() const;
    /** Returns a vector containing all of the states contained in node. */
    std::vector<State const *> getStates() const;
    /** Returns the time at which the last change occurred. */
    double getTimeOfLastChange() const;

    /* -------------------- Tree-related getters  ---------------------- */
    /** Returns the action mapping for this node. */
    ActionMapping *getMapping() const;
    /** Returns the parent entry for this node. */
    ObservationMappingEntry *getParentEntry() const;
    /** Returns the history-derived information for this node. */
    HistoricalData *getHistoricalData() const;
    /** Returns the parent action node of this belief. */
    ActionNode *getParentActionNode() const;
    /** Returns the parent belief of this belief. */
    BeliefNode *getParentBelief() const;
    /** Returns the last observation received before this belief. */
    std::unique_ptr<Observation> getLastObservation() const;
    /** Returns the last action taken before this belief. */
    std::unique_ptr<Action> getLastAction() const;
    /** Returns the belief node child corresponding to the given action and
     * observation
     */
    BeliefNode *getChild(Action const &action, Observation const &obs) const;

    /* -------------------- Tree-related getters  ---------------------- */
    /** Returns the recommended action to take from this node. */
    std::unique_ptr<Action> getRecommendedAction() const;
    /** Returns the best q-value */
    double getQValue() const;
    /** Recalculates the q-value for this belief node. */
    void recalculate();

private:
    /* -------------- Particle management / sampling ---------------- */
    /** Adds the given history entry to this belief node. */
    void addParticle(HistoryEntry *newHistEntry);
    /** Removes the given history entry from this belief node. */
    void removeParticle(HistoryEntry *histEntry);

    /* -------------------- Tree-related setters  ---------------------- */
    /** Sets the mapping for this node. */
    void setMapping(std::unique_ptr<ActionMapping> mapping);
    /** Sets the estimator for this node. */
    void setEstimator(std::unique_ptr<BeliefQValueEstimator> estimator);
    /** Sets the history-derived information for this node. */
    void setHistoricalData(std::unique_ptr<HistoricalData> data);

    /* -------------------- Tree-related methods  ---------------------- */
    /** Adds a child for the given action and observation;
     * returns the child node, and a boolean which is true iff a new node was
     * actually created.
     */
    std::pair<BeliefNode *, bool> createOrGetChild(Solver *solver, Action const &action,
            Observation const &obs);

private:
    /** The ID of this node. */
    long id_;

    /** The depth of this node in the tree. */
    long depth_;

    /** The observation entry that is this node's parent / owner. */
    ObservationMappingEntry *parentEntry_;

    std::unique_ptr<HistoricalData> data_;

    /** The set of particles belonging to this node. */
    abt::RandomAccessSet<HistoryEntry *> particles_;
    /** The number of sequences that start at this node. */
    long nStartingSequences_;
    /** The time at which the last particle modification occurred. */
    double tLastChange_;

    /** A mapping of actions to action children for this node. */
    std::unique_ptr<ActionMapping> actionMap_;
    /** The estimator - determines the q-value and best action for this node. */
    std::unique_ptr<BeliefQValueEstimator> estimator_;
};
} /* namespace solver */

#endif /* SOLVER_BELIEFNODE_HPP_ */
