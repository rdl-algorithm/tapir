#ifndef HISTORYSEQUENCE_HPP
#define HISTORYSEQUENCE_HPP

#include <vector>                       // for vector
#include "ChangeType.hpp"               // for ChangeType
#include "Observation.hpp"              // for Observation
class HistoryEntry;
class State;
class StateInfo;

class HistorySequence {
  public:
    friend class Histories;
    friend class Solver;
    friend class TextSerializer;

    HistorySequence(long startDepth);
    HistorySequence(HistoryEntry *startEntry, long startDepth);
    ~HistorySequence();
    HistorySequence(HistorySequence const &) = delete;
    HistorySequence(HistorySequence &&) = delete;
    HistorySequence &operator=(HistorySequence const &) = delete;
    HistorySequence &operator=(HistorySequence &&) = delete;

    HistoryEntry *getFirstEntry();
    HistoryEntry *addEntry(long actId, Observation &obs, StateInfo *s);
    HistoryEntry *addEntry(StateInfo *s, long actId, Observation &obs,
                           double rew, double disc);
    HistoryEntry *addEntry(StateInfo *s, long actId, Observation &obs,
                           double rew, double disc, long atIdx);
    void addEntry(HistoryEntry *histEntry);

    std::vector<State const *> getStates();

    long getId() {
        return id;
    }
    void fixEntryId();

  private:
    HistorySequence();

    static long currId;

    long id;
    long startDepth, startAffectedIdx, endAffectedIdx;
    std::vector<HistoryEntry *> histSeq;

    ChangeType changeType;
};

#endif /* HISTORYSEQUENCE_HPP */
