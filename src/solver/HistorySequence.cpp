#include "HistorySequence.hpp"

#include <limits>

#include <memory>                       // for unique_ptr
#include <utility>                      // for move
#include <vector>                       // for vector, __alloc_traits<>::value_type

#include "global.hpp"                     // for make_unique

#include "abstract-problem/Action.hpp"                   // for Action
#include "abstract-problem/Observation.hpp"              // for Observation

#include "changes/ChangeFlags.hpp"               // for ChangeFlags, ChangeFlags::UNCHANGED

#include "BeliefNode.hpp"
#include "BeliefTree.hpp"
#include "HistoryEntry.hpp"             // for HistoryEntry
#include "StateInfo.hpp"                // for StateInfo

namespace solver {
HistorySequence::HistorySequence() :
    HistorySequence(0, -1) {
}

HistorySequence::HistorySequence(long startDepth, long id) :
    id_(id),
    startDepth_(startDepth),
    histSeq_(),
    startAffectedIdx_(std::numeric_limits<long>::max()),
    endAffectedIdx_(-1),
    changeFlags_(ChangeFlags::UNCHANGED) {
}

// Do nothing!
HistorySequence::~HistorySequence() {
}

bool HistorySequence::testBackup(bool backingUp) {
    HistoryEntry *lastEntry = getLastEntry();
    if (lastEntry->getAction() != nullptr) {
        debug::show_message("ERROR: End of sequence has an action!?");
        return false;
    }

    for (std::unique_ptr<HistoryEntry> &entry : histSeq_) {
        if (!entry->isRegisteredAsParticle()
                || entry->associatedBeliefNode_ == nullptr) {
            debug::show_message("ERROR: Attempted to backup, but no belief"
                    " node is associated with this entry!");
            return false;
        }
        if (!backingUp && !entry->hasBeenBackedUp_) {
            debug::show_message("ERROR: Undoing backup, but it's already undone!");
            return false;
        } else if (backingUp && entry->hasBeenBackedUp_) {
            debug::show_message("ERROR: Doing backup, but it's already done!");
            return false;
        }
    }
    return true;
}

/* ----------- Methods to add or remove history entries ------------- */
void HistorySequence::reset() {
    for (std::unique_ptr<HistoryEntry> &entry : histSeq_) {
        entry->registerState(nullptr);
        if (entry->isRegisteredAsParticle()) {
            debug::show_message("ERROR: sequence should be fully deregistered"
                    " before deletion.");
        }
    }
    histSeq_.clear();
}

HistoryEntry *HistorySequence::addEntry(StateInfo *stateInfo) {
    std::unique_ptr<HistoryEntry> newEntry = std::make_unique<HistoryEntry>(
                stateInfo, this, histSeq_.size());
    HistoryEntry *newEntryReturn = newEntry.get();
    histSeq_.push_back(std::move(newEntry));
    return newEntryReturn;
}

HistoryEntry *HistorySequence::addEntry(StateInfo *stateInfo,
        Action const &action, Observation const &obs, double immediateReward) {
    std::unique_ptr<HistoryEntry> newEntry = std::make_unique<HistoryEntry>(
                stateInfo, this, histSeq_.size());
    newEntry->action_ = action.copy();
    newEntry->observation_ = obs.copy();
    newEntry->reward_ = immediateReward;
    HistoryEntry *newEntryReturn = newEntry.get();
    histSeq_.push_back(std::move(newEntry));
    return newEntryReturn;
}

HistoryEntry *HistorySequence::insertEntry(long index,
        StateInfo *stateInfo,
        Action const &action, Observation const &obs,
        double immediateReward) {
    std::unique_ptr<HistoryEntry> newEntry = std::make_unique<HistoryEntry>(
                stateInfo, this, histSeq_.size());
    newEntry->action_ = action.copy();
    newEntry->observation_ = obs.copy();
    newEntry->reward_ = immediateReward;
    HistoryEntry *newEntryReturn = newEntry.get();
    histSeq_.insert(histSeq_.begin() + index, std::move(newEntry));
    return newEntryReturn;
}

/* ------------------ Simple setters ------------------- */
void HistorySequence::setId(long id) {
    id_ = id;
}
/* ------------------ Simple getters ------------------- */
long HistorySequence::getId() const {
    return id_;
}
long HistorySequence::getStartDepth() const {
    return startDepth_;
}
long HistorySequence::getLength() const {
    return histSeq_.size();
}
HistoryEntry *HistorySequence::getEntry(long entryId) const {
    return histSeq_[entryId].get();
}
HistoryEntry *HistorySequence::getFirstEntry() const {
    return histSeq_.begin()->get();
}
HistoryEntry *HistorySequence::getLastEntry() const {
    return histSeq_.rbegin()->get();
}
std::vector<State const *> HistorySequence::getStates() const {
    std::vector<State const *> states;
    for (std::unique_ptr<HistoryEntry> const &entry : histSeq_) {
        states.push_back(entry->getState());
    }
    return states;
}

void HistorySequence::registerWith(BeliefNode *startNode, BeliefTree *policy) {
    bool registering = (startNode != nullptr);
    std::vector<std::unique_ptr<HistoryEntry>>::iterator historyIterator = histSeq_.begin();
    if (registering && (*historyIterator)->isRegisteredAsParticle()) {
        debug::show_message("WARNING: Already registered this seqeuence!");
        registerWith(nullptr, policy);
        registerWith(startNode, policy);
    }
    BeliefNode *currentNode = startNode;
    for (; historyIterator != histSeq_.end(); historyIterator++) {
        HistoryEntry *entry = historyIterator->get();
        if (registering) {
            entry->registerNode(currentNode);
            currentNode = policy->createOrGetChild(currentNode,
                    *entry->action_, *entry->observation_);
        } else {
            entry->registerNode(nullptr);
        }
    }
}

/* -------------- Change flagging methods ---------------- */
void HistorySequence::resetChangeFlags() {
    changeFlags_ = ChangeFlags::UNCHANGED;
    for (std::unique_ptr<HistoryEntry> &entry : histSeq_) {
        entry->resetChangeFlags();
    }
    resetAffectedIndices();
}

void HistorySequence::setChangeFlags(long index, ChangeFlags flags) {
    setChangeFlags(flags);
    getEntry(index)->setChangeFlags(flags);
    addAffectedIndex(index);
}

// These are private and ought not to be called outside HistorySequence.
void HistorySequence::setChangeFlags(ChangeFlags flags) {
    changeFlags_ |= flags;
}

void HistorySequence::resetAffectedIndices() {
    startAffectedIdx_ = std::numeric_limits<long>::max();
    endAffectedIdx_ = -1;
}

void HistorySequence::addAffectedIndex(long index) {
    if (startAffectedIdx_ > index) {
        startAffectedIdx_ = index;
    }
    if (endAffectedIdx_ < index) {
        endAffectedIdx_ = index;
    }
}
} /* namespace solver */
