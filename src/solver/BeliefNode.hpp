#ifndef SOLVER_BELIEFNODE_HPP_
#define SOLVER_BELIEFNODE_HPP_

#include <functional>
#include <map>                          // for map, map<>::value_compare
#include <memory>                       // for unique_ptr
#include <set>
#include <utility>                      // for pair

#include "global.hpp"                     // for RandomGenerator
#include "RandomAccessSet.hpp"

#include "solver/abstract-problem/Action.hpp"                   // for Action
#include "solver/abstract-problem/HistoricalData.hpp"
#include "solver/abstract-problem/State.hpp"
#include "solver/abstract-problem/Observation.hpp"              // for Observation

#include "solver/mappings/actions/ActionMapping.hpp"

namespace solver {
class BaseCachedValue;
template <typename T> class CachedValue;
typedef std::function<std::unique_ptr<Action>(BeliefNode const *)> ActionFunction;

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

    /* -------------------- Simple setters  ---------------------- */
    /** Sets the time of the last change to the current time. */
    void updateTimeOfLastChange();

    /* ----------------- Management of cached values ------------------- */
    /** Adds a value to be cached by this belief node. */
    BaseCachedValue *addCachedValue(std::unique_ptr<BaseCachedValue> value);
    /** Removes a value cached by this belief node. */
    void removeCachedValue(BaseCachedValue *value);

    /* ------------ Control of Q-value calculation and action selection -------------- */
    /** Sets the way in which the q-value for this belief node will be calculated. */
    void setQEstimator(CachedValue<double> *qEstimator);
    /** Sets the way in which the recommended action for this belief node will be chosen. */
    void setActionChooser(ActionFunction actionChooser);

    /* ------------ Q-value calculation and action selection -------------- */
    /** Returns the recommended action to take from this node. */
    std::unique_ptr<Action> getRecommendedAction() const;
    /** Returns the best q-value */
    double getQValue() const;
    /** Recalculates the q-value for this belief node. */
    void recalculateQValue();

private:
    /* -------------- Particle management / sampling ---------------- */
    /** Adds the given history entry to this belief node. */
    void addParticle(HistoryEntry *newHistEntry);
    /** Removes the given history entry from this belief node. */
    void removeParticle(HistoryEntry *histEntry);

    /* -------------------- Tree-related setters  ---------------------- */
    /** Sets the mapping for this node. */
    void setMapping(std::unique_ptr<ActionMapping> mapping);
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

    /** The cached values for this belief node. */
    std::unordered_map<BaseCachedValue const *, std::unique_ptr<BaseCachedValue>> cachedValues_;

    /** Determines the q-value for this node. */
    CachedValue<double> *qEstimator_;

    /** Determines the best action for this node. */
    ActionFunction actionChooser_;
};
} /* namespace solver */

#endif /* SOLVER_BELIEFNODE_HPP_ */
