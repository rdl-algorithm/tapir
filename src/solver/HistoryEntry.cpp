#include "HistoryEntry.hpp"

#include "StateInfo.hpp"                // for StateInfo

namespace solver {
HistoryEntry::HistoryEntry() :
    HistoryEntry(nullptr, 0, 0) {
}

HistoryEntry::HistoryEntry(StateInfo *stateInfo) :
    HistoryEntry(stateInfo, 0, 0) {
}

HistoryEntry::HistoryEntry(StateInfo *stateInfo, long seqId, long entryId) :
    stateInfo_(stateInfo),
    action_(-1),
    observation_(),
    hasBeenBackedUp_(false),
    seqId_(seqId),
    entryId_(entryId),
    discount_(1.0),
    immediateReward_(0),
    totalDiscountedReward_(0),
    owningBeliefNode_(nullptr) {
}

State *HistoryEntry::getState() {
    return stateInfo_->getState();
}
} /* namespace solver */
