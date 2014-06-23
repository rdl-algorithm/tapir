#include "Histories.hpp"

#include <utility>                      // for move

#include "global.hpp"                     // for make_unique
#include "HistoryEntry.hpp"             // for HistoryEntry
#include "HistorySequence.hpp"          // for HistorySequence

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
void Histories::deleteSequence(long seqId) {
    HistorySequence *sequence = getSequence(seqId);
    // Deregister and clear the sequence.
    sequence->erase();
    if (seqId < static_cast<long>(sequencesById_.size()) - 1) {
        sequencesById_[seqId] = std::move(sequencesById_[sequencesById_.size()-1]);
        sequencesById_[seqId]->id_ = seqId;
    }
    sequencesById_.pop_back();
}
} /* namespace solver */
