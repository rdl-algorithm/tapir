#include "Histories.hpp"

#include "HistorySequence.hpp"          // for HistorySequence
Histories::Histories() :
    allHistSeq() {
}

void Histories::reset() {
    allHistSeq.clear();
}

void Histories::add(std::unique_ptr<HistorySequence> histSeq) {
    allHistSeq.push_back(std::move(histSeq));
}

HistoryEntry *Histories::getHistoryEntry(long seqId, long entryId) {
    return allHistSeq[seqId]->get(entryId);
}

HistorySequence *Histories::getHistorySequence(long seqId) {
    return allHistSeq[seqId].get();
}
