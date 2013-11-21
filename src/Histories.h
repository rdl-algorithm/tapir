#ifndef HISTORIES_H
#define HISTORIES_H

#include <fstream>
#include <vector>

#include "HistorySequence.h"
#include "StateWrapper.h"
#include "StatePool.h"

class Histories {
public:
    friend class Solver;

    Histories();
    ~Histories();

    HistorySequence* addHistorySeq(StateWrapper *s);
    HistorySequence* addHistorySeq(StateWrapper *s, long startDepth);
    void readHistories(std::ifstream &inFile, StatePool *stPool);
    void write(std::ostream &os);

    inline void add(HistorySequence *histSeq) {
        allHistSeq.push_back(histSeq);
    }
    inline long getNHistSeq() {
        return allHistSeq.size();
    }
    inline HistoryEntry* getHistoryEntry(long seqId, long entryId) {
        return allHistSeq[seqId]->histSeq[entryId];
    }

private:
    std::vector<HistorySequence*> allHistSeq;
};
#endif
