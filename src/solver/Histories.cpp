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

/** Returns the number of history sequences. */
long Histories::getNumberOfSequences() const {
    return sequencesById_.size();
}

HistorySequence *Histories::addSequence(long startDepth) {
    std::unique_ptr<HistorySequence> histSeq(
            std::make_unique<HistorySequence>(startDepth,
                    sequencesById_.size()));
    HistorySequence *rawPtr = histSeq.get();
    sequencesById_.push_back(std::move(histSeq));
    return rawPtr;
}

HistorySequence *Histories::getSequence(long seqId) const {
    return sequencesById_[seqId].get();
}

void Histories::deleteSequence(long seqId) {
    HistorySequence *sequence = getSequence(seqId);
    sequence->reset();
    if (seqId < static_cast<long>(sequencesById_.size()) - 1) {
        sequencesById_[seqId] = std::move(sequencesById_[sequencesById_.size()-1]);
        sequencesById_[seqId]->setId(seqId);
    }
    sequencesById_.pop_back();
}

HistoryEntry *Histories::getEntry(long seqId, long entryId) const {
    return sequencesById_[seqId]->getEntry(entryId);
}
} /* namespace solver */
