#ifndef HISTORYSEQUENCE_HPP
#define HISTORYSEQUENCE_HPP

#include <memory>                       // for unique_ptr
#include <vector>                       // for vector

#include "Action.hpp"                   // for Action
#include "ChangeType.hpp"               // for ChangeType
#include "Observation.hpp"              // for Observation

class HistoryEntry;
class State;
class StateInfo;

class HistorySequence {
  public:
    friend class Solver;
    friend class TextSerializer;

    HistorySequence(long startDepth);
    HistorySequence(std::unique_ptr<HistoryEntry> startEntry, long startDepth);
    ~HistorySequence();

    /** Copying and moving is disallowed. */
    HistorySequence(HistorySequence const &) = delete;
    HistorySequence(HistorySequence &&) = delete;
    HistorySequence &operator=(HistorySequence const &) = delete;
    HistorySequence &operator=(HistorySequence &&) = delete;

    HistoryEntry *getFirstEntry();
    HistoryEntry *addEntry(StateInfo *stateInfo);
    HistoryEntry *addEntry(StateInfo *stateInfo, Action const &action,
            Observation const &obs, double immediateReward, double discount);
    HistoryEntry *addEntry(StateInfo *stateInfo, Action const &action,
            Observation const &obs, double immediateReward, double discount,
            long index);

    HistoryEntry *get(int entryId);

    std::vector<State const *> getStates();

    long getId() {
        return id;
    }

    void fixEntryIds();

  private:
    void addEntry(std::unique_ptr<HistoryEntry> histEntry);
    HistorySequence();

    static long currId;

    long id;
    long startDepth, startAffectedIdx, endAffectedIdx;
    std::vector<std::unique_ptr<HistoryEntry>> histSeq;

    ChangeType changeType;
};

#endif /* HISTORYSEQUENCE_HPP */
