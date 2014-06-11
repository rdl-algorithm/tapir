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
    friend class BasicSearchStrategy;
    friend class DefaultHistoryCorrector;
    friend class Histories;
    friend class Simulator;
    friend class Solver;
    friend class TextSerializer;

    /** Constructs an empty history sequence, with no ID assigned. */
    HistorySequence();
    /** Constructs an empty history sequence, assigning the given ID. */
    HistorySequence(long id);

    // Default destructor; copying and moving disallowed!
    ~HistorySequence();
    _NO_COPY_OR_MOVE(HistorySequence);

    /* ------------------ Simple getters ------------------- */
    /** Returns the ID of this sequence. */
    long getId() const;
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

  private:
    /** A method that verifies the validity of this sequence - this shouldn't
     * be necessary.
     */
    bool testBackup(bool backingUp);

    /* ----------- Methods to add or remove history entries ------------- */
    /** Erases all of the entries in this sequence, starting from firstEntryId. */
    void erase(long firstEntryId = 0);
    /** Adds a new entry with the given state info. */
    HistoryEntry *addEntry(StateInfo *stateInfo);

    /* -------------- Registration methods ---------------- */
    /** Registers the sequence with the given starting belief node. */
    void registerWith(BeliefNode *startNode, BeliefTree *policy);

    /* -------------- Change flagging methods ---------------- */
    /** Resets the changes for this sequence and all its entries. */
    void resetChangeFlags();
    /** Sets the given entry as having the given flags. */
    void setChangeFlags(long index, ChangeFlags flags);
    /** Sets the given change flags for this sequence. */
    void setChangeFlags(ChangeFlags flags);
    /** Resets the range affected indices for this sequence. */
    void resetAffectedIndices();
    /** Adds the given index as one of those affected by changes. */
    void addAffectedIndex(long index);

  private:
    /** The ID of this sequence. */
    long id_;

    /** The actual sequence of history entries. */
    std::vector<std::unique_ptr<HistoryEntry>> entrySequence_;

    /** The start and end of where this sequence is affected by changes. */
    long startAffectedIdx_, endAffectedIdx_;
    /** The types of changes that have affected this sequence. */
    ChangeFlags changeFlags_;
};
} /* namespace solver */

#endif /* SOLVER_HISTORYSEQUENCE_HPP_ */
