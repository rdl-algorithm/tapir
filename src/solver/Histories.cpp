#include "Histories.hpp"

#include <utility>                      // for move

#include "global.hpp"                     // for make_unique
#include "HistoryEntry.hpp"             // for HistoryEntry
#include "HistorySequence.hpp"          // for HistorySequence

namespace solver {
Histories::Histories() :
    allHistSeq_() {
}

void Histories::reset() {
    allHistSeq_.clear();
}

HistorySequence *Histories::addNew(long startDepth) {
    std::unique_ptr<HistorySequence> histSeq(
            std::make_unique<HistorySequence>(startDepth));
    HistorySequence *rawPtr = histSeq.get();
    allHistSeq_.push_back(std::move(histSeq));
    return rawPtr;
}

HistorySequence *Histories::getHistorySequence(long seqId) const {
    return allHistSeq_[seqId].get();
}

void Histories::deleteHistorySequence(long seqId) {
    HistorySequence *sequence = getHistorySequence(seqId);
    for (std::unique_ptr<HistoryEntry> &entry : sequence->histSeq_) {
        entry->registerState(nullptr);
        entry->registerNode(nullptr);
    }
    if (seqId < static_cast<long>(allHistSeq_.size()) - 1) {
        (allHistSeq_.begin() + seqId)->reset(allHistSeq_.rbegin()->release());
        (allHistSeq_.begin() + seqId)->get()->id_ = seqId;
    }
    allHistSeq_.pop_back();
}

HistoryEntry *Histories::getHistoryEntry(long seqId, long entryId) const {
    return allHistSeq_[seqId]->getEntry(entryId);
}
} /* namespace solver */
