#ifndef HISTORIES_HPP
#define HISTORIES_HPP

#include <vector>

#include "HistorySequence.hpp"
class HistoryEntry;

class Histories {
public:
    friend class Solver;
    friend class TextSerializer;

    Histories();
    ~Histories();
    Histories(const Histories&) = delete;
    Histories(Histories&) = delete;
    Histories &operator=(const Histories&) = delete;
    Histories &operator=(Histories&) = delete;

    void reset();

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