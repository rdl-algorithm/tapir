#ifndef HISTORYSEQUENCE_HPP
#define HISTORYSEQUENCE_HPP

#include <vector>                       // for vector
#include "ChangeType.hpp"               // for ChangeType
#include "Observation.hpp"              // for Observation
class HistoryEntry;
class State;
class StateWrapper;

class HistorySequence {
public:
    friend class Histories;
    friend class Solver;
    friend class TextSerializer;

    HistorySequence();
    HistorySequence(long startDepth);
    HistorySequence(HistoryEntry* startEntry, long startDepth);
    ~HistorySequence();
    HistorySequence(const HistorySequence&) = delete;
    HistorySequence(HistorySequence&) = delete;
    HistorySequence &operator=(const HistorySequence&) = delete;
    HistorySequence &operator=(HistorySequence&) = delete;

    HistoryEntry* getFirstEntry();
    HistoryEntry* addEntry(long actId, Observation &obs, StateWrapper *s);
    HistoryEntry* addEntry(StateWrapper *s, long actId, Observation &obs,
            double rew, double disc);
    HistoryEntry* addEntry(StateWrapper *s, long actId, Observation &obs,
            double rew, double disc, long atIdx);
    void addEntry(HistoryEntry *histEntry);

    void getStValsSeq(std::vector<State> &seqStVals);

    long getId() {
        return id;
    }
    void fixEntryId();

private:
    static long currId;

    long id;
    long startDepth, startAffectedIdx, endAffectedIdx;
    std::vector<HistoryEntry*> histSeq;

    ChangeType chType;
};

#endif /* HISTORYSEQUENCE_HPP */
