#ifndef HISTORIES_HPP
#define HISTORIES_HPP

#include <vector>                       // for vector
#include "HistorySequence.hpp"          // for HistorySequence
class HistoryEntry;

class Histories {
  public:
    friend class TextSerializer;

    Histories();
    ~Histories() = default;
    Histories(Histories const &) = delete;
    Histories(Histories &&) = delete;
    Histories &operator=(Histories const &) = delete;
    Histories &operator=(Histories &&) = delete;

    void reset();
    void add(std::unique_ptr<HistorySequence> histSeq);
    HistorySequence *getHistorySequence(long seqId);
    HistoryEntry *getHistoryEntry(long seqId, long entryId);

    long getNHistSeq() {
        return allHistSeq.size();
    }

  private:
    std::vector<std::unique_ptr<HistorySequence> > allHistSeq;
};

#endif /* HISTORIES_HPP */
