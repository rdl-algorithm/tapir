#include "Histories.hpp"

#include <utility>                      // for move

#include "HistorySequence.hpp"          // for HistorySequence

namespace solver {
Histories::Histories() :
    allHistSeq_() {
}

void Histories::reset() {
    allHistSeq_.clear();
}

void Histories::add(std::unique_ptr<HistorySequence> histSeq) {
    allHistSeq_.push_back(std::move(histSeq));
}

HistoryEntry *Histories::getHistoryEntry(long seqId, long entryId) {
    return allHistSeq_[seqId]->get(entryId);
}

HistorySequence *Histories::getHistorySequence(long seqId) {
    return allHistSeq_[seqId].get();
}
} /* namespace solver */
