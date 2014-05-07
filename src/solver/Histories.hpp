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
    friend class Solver;
    friend class TextSerializer;

    /** Constructs an empty bundle of histories. */
    Histories();

    // Default destructor; copying and moving disallowed!
    ~Histories() = default;
    _NO_COPY_OR_MOVE(Histories);

    /* ------------------- Retrieving sequences ------------------- */
    /** Returns the number of history sequences. */
    long getNumberOfSequences() const;
    /** Retrieves the history sequence with the given ID. */
    HistorySequence *getSequence(long seqId) const;


  private:
    /* ---------------- Adding / removing sequences  ---------------- */
    /** Resets the histories to be empty. */
    void reset();
    /** Adds a new history sequence. */
    HistorySequence *createSequence();
    /** Deletes the history sequence with the given ID. */
    void deleteSequence(long seqId);


  private:
    std::vector<std::unique_ptr<HistorySequence>> sequencesById_;
};
} /* namespace solver */

#endif /* SOLVER_HISTORIES_HPP_ */
