#ifndef HISTORIES_HPP
#define HISTORIES_HPP

#include <iosfwd>
#include <vector>

#include "HistorySequence.hpp"
class HistoryEntry;
class StatePool;
class StateWrapper;

class Histories {
public:
    friend class Solver;

    Histories();
    ~Histories();
    Histories(const Histories&) = delete;
    Histories(Histories&) = delete;
    Histories &operator=(const Histories&) = delete;
    Histories &operator=(Histories&) = delete;

    HistorySequence* addHistorySeq(StateWrapper *s);
    HistorySequence* addHistorySeq(StateWrapper *s, long startDepth);
    void readHistories(std::ifstream &inFile, StatePool *stPool);
    void write(std::ostream &os);

    void add(HistorySequence *histSeq) {
        allHistSeq.push_back(histSeq);
    }
    long getNHistSeq() {
        return allHistSeq.size();
    }
    HistoryEntry* getHistoryEntry(long seqId, long entryId) {
        return allHistSeq[seqId]->histSeq[entryId];
    }

private:
    std::vector<HistorySequence*> allHistSeq;
};

#endif /* HISTORIES_HPP */
