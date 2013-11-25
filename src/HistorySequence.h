#ifndef HISTORYSEQUENCE_H
#define HISTORYSEQUENCE_H

#include <iostream>
#include <vector>

#include "HistoryEntry.h"
#include "Model.h"

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
    HistoryEntry* addEntry(long actId, ObsVals &obs, StateWrapper *s);
    HistoryEntry* addEntry(StateWrapper *s, long actId, ObsVals &obs,
            double rew, double disc);
    HistoryEntry* addEntry(StateWrapper *s, long actId, ObsVals &obs,
            double rew, double disc, long atIdx);
    void addEntry(HistoryEntry *histEntry);

    void updateVal(Model *m);
    void getStValsSeq(std::vector<StateVals> &seqStVals);

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

    Change chType;
};
#endif
