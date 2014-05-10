#include "HistoryEntry.hpp"

#include <iostream>

#include "BeliefNode.hpp"
#include "changes/ChangeFlags.hpp"              // for ChangeFlags
#include "StateInfo.hpp"                // for StateInfo

namespace solver {
class HistorySequence;

HistoryEntry::HistoryEntry() :
    HistoryEntry(nullptr, 0) {
}

HistoryEntry::HistoryEntry(HistorySequence* owningSequence, long entryId) :
    stateInfo_(nullptr),
    action_(nullptr),
    transitionParameters_(nullptr),
    observation_(nullptr),
    hasBeenBackedUp_(false),
    entryId_(entryId),
    reward_(0),
    rewardFromHere_(0),
    owningSequence_(owningSequence),
    associatedBeliefNode_(nullptr),
    isRegisteredAsParticle_(false),
    changeFlags_(ChangeFlags::UNCHANGED) {
}

HistoryEntry::HistoryEntry(StateInfo *stateInfo,
        HistorySequence* owningSequence, long entryId) :
                HistoryEntry(owningSequence, entryId) {
    registerState(stateInfo);
}

HistoryEntry::~HistoryEntry() {
}

/* ----------------- Simple getters ------------------- */
long HistoryEntry::getId() const {
    return entryId_;
}
double HistoryEntry::getReward() const {
    return reward_;
}
double HistoryEntry::getCumulativeReward() const {
    return rewardFromHere_;
}
State const *HistoryEntry::getState() const {
    return stateInfo_->getState();
}
StateInfo const *HistoryEntry::getStateInfo() const {
    return stateInfo_;
}
Action const *HistoryEntry::getAction() const{
    return action_.get();
}
Observation const *HistoryEntry::getObservation() const {
    return observation_.get();
}
TransitionParameters const *HistoryEntry::getTransitionParameters() const {
    return transitionParameters_.get();
}
BeliefNode *HistoryEntry::getAssociatedBeliefNode() const {
    return associatedBeliefNode_;
}

/* -------------- Registration methods ---------------- */
bool HistoryEntry::isRegisteredAsParticle() const {
    return isRegisteredAsParticle_;
}


/* ============================ PRIVATE ============================ */


/* -------------- Registration methods ---------------- */
void HistoryEntry::registerNode(BeliefNode *node) {
    // If it's registered, we deregister it.
    if (isRegisteredAsParticle_) {
        isRegisteredAsParticle_ = false;
        associatedBeliefNode_->removeParticle(this);
    }
    // Then we register it with the new node.
    if (node != nullptr) {
        isRegisteredAsParticle_ = true;
        associatedBeliefNode_ = node;
        associatedBeliefNode_->addParticle(this);
    }
}
void HistoryEntry::registerState(StateInfo *info) {
    if (stateInfo_ == info) {
        return;
    }
    if (stateInfo_ != nullptr) {
        stateInfo_->removeHistoryEntry(this);
        stateInfo_ = nullptr;
    }
    if (info != nullptr) {
        stateInfo_ = info;
        stateInfo_->addHistoryEntry(this);
    }
}

/* ----------------- Change flagging ------------------- */
void HistoryEntry::resetChangeFlags() {
    changeFlags_ = ChangeFlags::UNCHANGED;
}
void HistoryEntry::setChangeFlags(ChangeFlags flags) {
    changeFlags_ |= flags;
}
} /* namespace solver */
