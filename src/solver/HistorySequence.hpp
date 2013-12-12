#ifndef SOLVER_HISTORYSEQUENCE_HPP_
#define SOLVER_HISTORYSEQUENCE_HPP_

#include <memory>                       // for unique_ptr
#include <vector>                       // for vector

#include "Action.hpp"                   // for Action
#include "ChangeType.hpp"               // for ChangeType
#include "Observation.hpp"              // for Observation

namespace solver {
class HistoryEntry;
class State;
class StateInfo;

class HistorySequence {
  public:
    friend class Solver;
    friend class TextSerializer;

    /** Constructs a history sequence starting at the given depth. */
    HistorySequence(long startDepth);
    /** Constructs a history sequence with the given initial entry, starting
     * at the given depth.
     */
    HistorySequence(std::unique_ptr<HistoryEntry> startEntry, long startDepth);

    /** Default destructor. */
    ~HistorySequence();

    /** Copying and moving is disallowed. */
    HistorySequence(HistorySequence const &) = delete;
    HistorySequence(HistorySequence &&) = delete;
    HistorySequence &operator=(HistorySequence const &) = delete;
    HistorySequence &operator=(HistorySequence &&) = delete;

    /** Returns the first entry in this sequence. */
    HistoryEntry *getFirstEntry();
    /** Adds a new entry with the given state info. */
    HistoryEntry *addEntry(StateInfo *stateInfo);
    /** Adds a new entry with the given state info, action, observation,
     * immediate reward, and discount.
     */
    HistoryEntry *addEntry(StateInfo *stateInfo, Action const &action,
            Observation const &obs, double immediateReward, double discount);
    /** Adds a new entry at the specified position (index),
     * with the given state info, action, observation, immediate reward, and
     * discount.
     */
    HistoryEntry *addEntry(StateInfo *stateInfo, Action const &action,
            Observation const &obs, double immediateReward, double discount,
            long index);

    /** Returns the history entry in this sequence with the given ID. */
    HistoryEntry *get(int entryId);

    /** Returns the states in this sequence as a vector. */
    std::vector<State const *> getStates();

    /** Fixes the entry IDs to be sequential, starting at 0. */
    void fixEntryIds();

  private:
    /** Constructs a history sequence with no start depth or ID! */
    HistorySequence();
    /** Adds an already-created entry to the sequence. */
    void addEntry(std::unique_ptr<HistoryEntry> histEntry);

    /** The current ID; will be assigned to the next sequence to be made. */
    static long currId;

    /** The ID f of this sequence. */
    long id_;
    /** The starting depth of this sequence. */
    long startDepth_;

    /** The actual sequence of history entries. */
    std::vector<std::unique_ptr<HistoryEntry>> histSeq_;

    /** The start and end of where this sequence is affected by changes. */
    long startAffectedIdx_, endAffectedIdx_;
    /** The type of change that affects this sequence. */
    ChangeType changeType_;
};
} /* namespace solver */

#endif /* SOLVER_HISTORYSEQUENCE_HPP_ */
