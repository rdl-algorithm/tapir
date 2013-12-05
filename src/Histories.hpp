#ifndef HISTORIES_HPP
#define HISTORIES_HPP

#include <vector>                       // for vector
#include "HistorySequence.hpp"          // for HistorySequence
class HistoryEntry;

class Histories {
  public:
    friend class Solver;
    friend class TextSerializer;

    Histories();
    ~Histories();
    Histories(Histories const &) = delete;
    Histories(Histories &&) = delete;
    Histories &operator=(Histories const &) = delete;
    Histories &operator=(Histories &&) = delete;

    void reset();

    void add(HistorySequence *histSeq) {
        allHistSeq.push_back(histSeq);
    }
    long getNHistSeq() {
        return allHistSeq.size();
    }
    HistoryEntry *getHistoryEntry(long seqId, long entryId) {
        return allHistSeq[seqId]->histSeq[entryId];
    }

  private:
    std::vector<HistorySequence *> allHistSeq;
};

#endif /* HISTORIES_HPP */
