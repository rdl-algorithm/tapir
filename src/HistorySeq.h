#ifndef HISTORYSEQ_H
#define HISTORYSEQ_H

#include <iostream>
#include <vector>

#include "HistoryEntry.h"
#include "Model.h"

class HistorySeq {
public:
    friend class Histories;
    friend class Solver;

    HistorySeq(HistoryEntry* startEntry);
    HistorySeq(HistoryEntry* startEntry, long startDepth);
    ~HistorySeq();

    HistoryEntry* getFirstEntry();
    HistoryEntry* addEntry(long actId, ObsVals &obs, State *s);
    HistoryEntry* addEntry(State *s, long actId, ObsVals &obs, double rew,
            double disc);
    HistoryEntry* addEntry(State *s, long actId, ObsVals &obs, double rew,
            double disc, long atIdx);
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
    unsigned long startDepth, startAffectedIdx, endAffectedIdx;
    std::vector<HistoryEntry*> histSeq;

    Change chType;

};
#endif
