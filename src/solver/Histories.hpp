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

    // Default destructor; copying and moving disallowed!
    ~Histories() = default;
    Histories(Histories const &) = delete;
    Histories(Histories &&) = delete;
    Histories &operator=(Histories const &) = delete;
    Histories &operator=(Histories &&) = delete;

    /** Resets the histories to be empty. */
    void reset();
    /** Adds a new history sequence, starting at the given depth. */
    HistorySequence *addNew(long startDepth);
    /** Retrieves the history sequence with the given ID. */
    HistorySequence *getHistorySequence(long seqId) const;
    /** Deletes the history sequence with the given ID. */
    void deleteHistorySequence(long seqId);
    /** Retrieves the history entry with the given sequence ID and entry ID. */
    HistoryEntry *getHistoryEntry(long seqId, long entryId) const;

  private:
    std::vector<std::unique_ptr<HistorySequence>> allHistSeq_;
};
} /* namespace solver */

#endif /* SOLVER_HISTORIES_HPP_ */
