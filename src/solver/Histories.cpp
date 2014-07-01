/** file: Histories.cpp
 *
 * Contains the implementation of the Histories class
 */
#include "solver/Histories.hpp"

#include <utility>                      // for move

#include "global.hpp"                     // for make_unique

#include "solver/HistoryEntry.hpp"             // for HistoryEntry
#include "solver/HistorySequence.hpp"          // for HistorySequence

namespace solver {
Histories::Histories() :
        sequencesById_() {
}

/* ------------------- Retrieving sequences ------------------- */
long Histories::getNumberOfSequences() const {
    return sequencesById_.size();
}
HistorySequence *Histories::getSequence(long seqId) const {
    return sequencesById_[seqId].get();
}


/* ============================ PRIVATE ============================ */


/* ---------------- Adding / removing sequences  ---------------- */
void Histories::reset() {
    sequencesById_.clear();
}
HistorySequence *Histories::createSequence() {
    std::unique_ptr<HistorySequence> histSeq(
            std::make_unique<HistorySequence>(sequencesById_.size()));
    HistorySequence *rawPtr = histSeq.get();
    sequencesById_.push_back(std::move(histSeq));
    return rawPtr;
}
void Histories::deleteSequence(HistorySequence *sequence) {
    // Retrieve the current ID of the sequence, which should be its position in the vector.
    long seqId = sequence->id_;

    if (sequencesById_[seqId].get() != sequence) {
        debug::show_message("ERROR: sequence ID does not match its index!");
    }

    // Deregister and clear the sequence.
    sequence->erase();
    if (seqId < static_cast<long>(sequencesById_.size()) - 1) {
        sequencesById_[seqId] = std::move(sequencesById_[sequencesById_.size()-1]);
        sequencesById_[seqId]->id_ = seqId;
    }
    sequencesById_.pop_back();
}
} /* namespace solver */
