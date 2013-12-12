#ifndef SOLVER_HISTORIES_HPP_
#define SOLVER_HISTORIES_HPP_

#include <memory>                       // for unique_ptr
#include <vector>                       // for vector

namespace solver {
class HistoryEntry;
class HistorySequence;

class Histories {
  public:
    friend class TextSerializer;

    /** Constructs an empty bundle of histories. */
    Histories();
    /** Default destructor. */
    ~Histories() = default;

    /* Copying and moving is disallowed. */
    Histories(Histories const &) = delete;
    Histories(Histories &&) = delete;
    Histories &operator=(Histories const &) = delete;
    Histories &operator=(Histories &&) = delete;

    /** Resets the histories to be empty. */
    void reset();
    /** Adds a new history sequence. */
    void add(std::unique_ptr<HistorySequence> histSeq);
    /** Retrieves the history sequence with the given ID. */
    HistorySequence *getHistorySequence(long seqId);
    /** Retrieves the history entry with the given sequence ID and entry ID. */
    HistoryEntry *getHistoryEntry(long seqId, long entryId);

  private:
    std::vector<std::unique_ptr<HistorySequence>> allHistSeq_;
};
} /* namespace solver */

#endif /* SOLVER_HISTORIES_HPP_ */
