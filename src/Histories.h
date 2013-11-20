#ifndef HISTORIES_H
#define HISTORIES_H

#include <fstream>
#include <vector>

#include "HistorySeq.h"
#include "State.h"
#include "StatePool.h"

class Histories {
public:
    friend class Solver;

    Histories();
    ~Histories();

    HistorySeq* addHistorySeq(State *s);
    HistorySeq* addHistorySeq(State *s, long startDepth);
    void readHistories(std::ifstream &inFile, StatePool *stPool);
    void write(std::ostream &os);

    inline void add(HistorySeq *histSeq) {
        allHistSeq.push_back(histSeq);
    }
    inline long getNHistSeq() {
        return allHistSeq.size();
    }
    inline HistoryEntry* getHistoryEntry(long seqId, long entryId) {
        return allHistSeq[seqId]->histSeq[entryId];
    }

private:
    std::vector<HistorySeq*> allHistSeq;
};
#endif
