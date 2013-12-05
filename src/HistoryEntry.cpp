#include "HistoryEntry.hpp"

class BeliefNode;
class StateInfo;

HistoryEntry::HistoryEntry() :
    HistoryEntry(nullptr, 0, 0) {
}

HistoryEntry::HistoryEntry(StateInfo *stateInfo) :
    HistoryEntry(stateInfo, 0, 0) {
}

HistoryEntry::HistoryEntry(StateInfo *stateInfo, long seqId, long entryId) :
    stateInfo(stateInfo),
    hasBeenBackup(false),
    seqId(seqId),
    entryId(entryId),
    discount(1.0),
    immediateReward(0),
    qVal(0),
    actId(-1),
    obs(),
    partOfBelNode(nullptr) {
}

void HistoryEntry::setBelNode(BeliefNode *bel) {
    partOfBelNode = bel;
}
