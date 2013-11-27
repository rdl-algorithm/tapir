#ifndef HISTORYSEQUENCE_HPP
#define HISTORYSEQUENCE_HPP

#include <ostream>
#include <vector>

#include "ChangeType.hpp"
#include "Observation.hpp"
#include "State.hpp"
class HistoryEntry;
class StateWrapper;

class HistorySequence {
public:
    friend class Histories;
    friend class Solver;

    HistorySequence(HistoryEntry* startEntry);
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

    void write(std::ostream &os);
    void writeln(std::ostream &os);

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
