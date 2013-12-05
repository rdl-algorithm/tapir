#include "HistoryEntry.hpp"

class BeliefNode;
class StateWrapper;

HistoryEntry::HistoryEntry() :
    HistoryEntry(nullptr, 0, 0) {
}

HistoryEntry::HistoryEntry(StateWrapper *st) :
    HistoryEntry(st, 0, 0) {
}

HistoryEntry::HistoryEntry(StateWrapper *st, long seqId, long entryId) :
    st(st),
    hasBeenBackup(false),
    seqId(seqId),
    entryId(entryId),
    disc(1.0),
    rew(0),
    qVal(0),
    actId(-1),
    obs(),
    partOfBelNode(nullptr) {
}

void HistoryEntry::setBelNode(BeliefNode *bel) {
    partOfBelNode = bel;
}
