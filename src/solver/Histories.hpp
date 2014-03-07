#ifndef SOLVER_HISTORIES_HPP_
#define SOLVER_HISTORIES_HPP_

#include <map>                          // for map
#include <memory>                       // for unique_ptr

#include "global.hpp"

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
    _NO_COPY_OR_MOVE(Histories);

    /** Resets the histories to be empty. */
    void reset();
    /** Adds a new history sequence, starting at the given depth. */
    HistorySequence *addNew(long startDepth);
    /** Returns the number of history sequences. */
    long getNumberOfSequences() const;
    /** Retrieves the history sequence with the given ID. */
    HistorySequence *getHistorySequence(long seqId) const;
    /** Deletes the history sequence with the given ID. */
    void deleteHistorySequence(long seqId);
    /** Retrieves the history entry with the given sequence ID and entry ID. */
    HistoryEntry *getHistoryEntry(long seqId, long entryId) const;

  private:
    std::vector<std::unique_ptr<HistorySequence>> sequencesById_;
};
} /* namespace solver */

#endif /* SOLVER_HISTORIES_HPP_ */
