#ifndef SOLVER_HISTORYSEQUENCE_HPP_
#define SOLVER_HISTORYSEQUENCE_HPP_

#include <memory>                       // for unique_ptr
#include <vector>                       // for vector

#include "topology/Action.hpp"                   // for Action
#include "ChangeFlags.hpp"               // for ChangeFlags
#include "topology/Observation.hpp"              // for Observation
#include "topology/State.hpp"

#include "global.hpp"

namespace solver {
class HistoryEntry;
class StateInfo;

class HistorySequence {
  public:
    friend class Histories;
    friend class Solver;
    friend class TextSerializer;

    /** Constructs an empty history sequence starting at depth 0. */
    HistorySequence();
    /** Constructs an empty history sequence starting at the given depth. */
    HistorySequence(long startDepth);

    // Default destructor; copying and moving disallowed!
    ~HistorySequence();
    _NO_COPY_OR_MOVE(HistorySequence);

    /** Adds a new entry with the given state info and current discount. */
    HistoryEntry *addEntry(StateInfo *stateInfo, double discount);
    /** Adds a new entry with the given current discount, as
     * well as the given action,observation and immediate reward.
     */
    HistoryEntry *addEntry(StateInfo *stateInfo, double discount,
            Action const &action, Observation const &obs,
            double immediateReward);
    /** Adds a new entry at the specified position (index),
     * with the given state info, action, observation, immediate reward, and
     * discount.
     */
    HistoryEntry *insertEntry(long index,
            StateInfo *stateInfo, double discount,
            Action const &action, Observation const &obs,
            double immediateReward);

    /** Returns the history entry in this sequence with the given ID. */
    HistoryEntry *getEntry(long entryId) const;

    /** Returns the states in this sequence as a vector. */
    std::vector<State const *> getStates() const;

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

    /** The current ID; will be assigned to the next sequence to be made. */
    static long currId;

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
