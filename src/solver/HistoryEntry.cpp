#include "HistoryEntry.hpp"

#include "StateInfo.hpp"                // for StateInfo

HistoryEntry::HistoryEntry() :
    HistoryEntry(nullptr, 0, 0) {
}

HistoryEntry::HistoryEntry(StateInfo *stateInfo) :
    HistoryEntry(stateInfo, 0, 0) {
}

HistoryEntry::HistoryEntry(StateInfo *stateInfo, long seqId, long entryId) :
    stateInfo(stateInfo),
    action(-1),
    observation(),
    hasBeenBackedUp(false),
    seqId(seqId),
    entryId(entryId),
    discount(1.0),
    immediateReward(0),
    qVal(0),
    owningBeliefNode(nullptr) {
}

State *HistoryEntry::getState() {
    return stateInfo->getState();
}
