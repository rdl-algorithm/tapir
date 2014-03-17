#include "Histories.hpp"

#include <utility>                      // for move

#include "global.hpp"                     // for make_unique
#include "HistoryEntry.hpp"             // for HistoryEntry
#include "HistorySequence.hpp"          // for HistorySequence

namespace solver {
Histories::Histories() :
        sequencesById_() {
}

void Histories::reset() {
    sequencesById_.clear();
}

HistorySequence *Histories::addNew(long startDepth) {
    std::unique_ptr<HistorySequence> histSeq(
            std::make_unique<HistorySequence>(startDepth,
                    sequencesById_.size()));
    HistorySequence *rawPtr = histSeq.get();
    sequencesById_.push_back(std::move(histSeq));
    return rawPtr;
}

/** Returns the number of history sequences. */
long Histories::getNumberOfSequences() const {
    return sequencesById_.size();
}

HistorySequence *Histories::getHistorySequence(long seqId) const {
    return sequencesById_[seqId].get();
}

void Histories::deleteHistorySequence(long seqId) {
    HistorySequence *sequence = getHistorySequence(seqId);
    for (std::unique_ptr<HistoryEntry> &entry : sequence->histSeq_) {
        entry->registerState(nullptr);
        if (entry->isRegisteredAsParticle_) {
            debug::show_message("ERROR: sequence should be deregistered"
                    " before deletion.");
        }
    }
    if (seqId < static_cast<long>(sequencesById_.size()) - 1) {
        sequencesById_[seqId] = std::move(sequencesById_[sequencesById_.size()-1]);
        sequencesById_[seqId]->id_ = seqId;
    }
    sequencesById_.pop_back();
}

HistoryEntry *Histories::getHistoryEntry(long seqId, long entryId) const {
    return sequencesById_[seqId]->getEntry(entryId);
}
} /* namespace solver */
