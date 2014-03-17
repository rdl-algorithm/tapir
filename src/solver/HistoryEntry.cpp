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

State const *HistoryEntry::getState() const {
    return stateInfo_->getState();
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

void HistoryEntry::resetChangeFlags() {
    changeFlags_ = ChangeFlags::UNCHANGED;
}

void HistoryEntry::setChangeFlags(ChangeFlags flags) {
    changeFlags_ |= flags;
}

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
} /* namespace solver */
