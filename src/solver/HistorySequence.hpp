#ifndef SOLVER_HISTORYSEQUENCE_HPP_
#define SOLVER_HISTORYSEQUENCE_HPP_

#include <memory>                       // for unique_ptr
#include <vector>                       // for vector

#include "changes/ChangeFlags.hpp"               // for ChangeFlags

#include "abstract-problem/Action.hpp"                   // for Action
#include "abstract-problem/Observation.hpp"              // for Observation
#include "abstract-problem/State.hpp"

#include "global.hpp"

namespace solver {
class BeliefNode;
class BeliefTree;
class HistoryEntry;
class StateInfo;

class HistorySequence {
  public:
    friend class AbstractBackpropagationStrategy;
    friend class Solver;
    friend class TextSerializer;
    friend class DefaultHistoryCorrector;

    /** Constructs an empty history sequence starting at depth 0, without
     * assigning an ID.
     */
    HistorySequence();
    /** Constructs an empty history sequence starting at the given depth,
     * and assigning the given ID.
     */
    HistorySequence(long startDepth, long id);

    // Default destructor; copying and moving disallowed!
    ~HistorySequence();
    _NO_COPY_OR_MOVE(HistorySequence);

    /** A method that verifies the validity of this sequence - this shouldn't
     * be necessary.
     */
    bool testBackup(bool backingUp);

    /* ----------- Methods to add or remove history entries ------------- */
    /** Clears all the entries in the sequence. */
    void reset();
    /** Adds a new entry with the given state info. */
    HistoryEntry *addEntry(StateInfo *stateInfo);
    /** Adds a new entry with the given current discount, as
     * well as the given action,observation and immediate reward.
     */
    HistoryEntry *addEntry(StateInfo *stateInfo,
            Action const &action, Observation const &obs,
            double immediateReward);
    /** Adds a new entry at the specified position (index),
     * with the given state info, action, observation, immediate reward, and
     * discount.
     */
    HistoryEntry *insertEntry(long index,
            StateInfo *stateInfo,
            Action const &action, Observation const &obs,
            double immediateReward);

    /* ------------------ Simple setters ------------------- */
    /** Sets the ID of this sequence. */
    void setId(long id);

    /* ------------------ Simple getters ------------------- */
    /** Returns the ID of this sequence. */
    long getId() const;
    /** Returns the starting depth of this sequence. */
    long getStartDepth() const;
    /** Returns the length of this sequence. */
    long getLength() const;
    /** Returns the history entry in this sequence with the given ID. */
    HistoryEntry *getEntry(long entryId) const;
    /** Returns the first entry in this sequence. */
    HistoryEntry *getFirstEntry() const;
    /** Returns the last entry in this sequence. */
    HistoryEntry *getLastEntry() const;
    /** Returns the states in this sequence as a vector. */
    std::vector<State const *> getStates() const;

    /* -------------- Registration methods ---------------- */
    /** Registers the sequence with the given belief node. */
    void registerWith(BeliefNode *startNode,
            BeliefTree *policy);
    /** Changes the belief node that is considered to be the root of this
     *  sequence - use a value of nullptr to detach the sequence entirely.
     */
    //void registerStartingNode(BeliefNode *startNode);
    /** Registers or deregisters the sequence, apart from its starting
     * belief node.
     * If registering, the policy must be supplied as the creation of
     * new belief nodes may be required.
     */
    //void registerRestOfSequence(bool registering, BeliefTree *policy);

    /* -------------- Change flagging methods ---------------- */
    /** Resets the changes for this sequence and all its entries. */
    void resetChangeFlags();
    /** Sets the given entry as having the given flags. */
    void setChangeFlags(long index, ChangeFlags flags);
private:
    /** Sets the given change flags for this sequence. */
    void setChangeFlags(ChangeFlags flags);
    /** Resets the range affected indices for this sequence. */
    void resetAffectedIndices();
    /** Adds the given index as one of those affected by changes. */
    void addAffectedIndex(long index);

    /** The ID of this sequence. */
    long id_;
    /** The starting depth of this sequence. */
    long startDepth_;

    /** The actual sequence of history entries. */
    std::vector<std::unique_ptr<HistoryEntry>> histSeq_;

    /** The start and end of where this sequence is affected by changes. */
    long startAffectedIdx_, endAffectedIdx_;
    /** The type of change that affects this sequence. */
    ChangeFlags changeFlags_;
};
} /* namespace solver */

#endif /* SOLVER_HISTORYSEQUENCE_HPP_ */
