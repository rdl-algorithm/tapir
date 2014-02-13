#include "HistorySequence.hpp"

#include <limits>

#include <memory>                       // for unique_ptr
#include <utility>                      // for move
#include <vector>                       // for vector, __alloc_traits<>::value_type

#include "defs.hpp"                     // for make_unique

#include "Action.hpp"                   // for Action
#include "ChangeFlags.hpp"               // for ChangeFlags, ChangeFlags::UNCHANGED
#include "HistoryEntry.hpp"             // for HistoryEntry
#include "Observation.hpp"              // for Observation
#include "StateInfo.hpp"                // for StateInfo

namespace solver {
long HistorySequence::currId = 0;

HistorySequence::HistorySequence() :
    HistorySequence(0) {
}

HistorySequence::HistorySequence(long startDepth) :
    id_(currId),
    startDepth_(startDepth),
    histSeq_(),
    startAffectedIdx_(std::numeric_limits<long>::max()),
    endAffectedIdx_(-1),
    changeFlags_(ChangeFlags::UNCHANGED) {
    currId++;
}

// Do nothing!
HistorySequence::~HistorySequence() {
}

HistoryEntry *HistorySequence::addEntry(StateInfo *stateInfo, double discount) {
    std::unique_ptr<HistoryEntry> newEntry = std::make_unique<HistoryEntry>(
                stateInfo, discount, this, histSeq_.size());
    HistoryEntry *newEntryReturn = newEntry.get();
    histSeq_.push_back(std::move(newEntry));
    return newEntryReturn;
}

HistoryEntry *HistorySequence::addEntry(StateInfo *stateInfo, double discount,
        Action const &action, Observation const &obs, double immediateReward) {
    std::unique_ptr<HistoryEntry> newEntry = std::make_unique<HistoryEntry>(
                stateInfo, discount, this, histSeq_.size());
    newEntry->action_ = action;
    newEntry->observation_ = obs.copy();
    newEntry->immediateReward_ = immediateReward;
    HistoryEntry *newEntryReturn = newEntry.get();
    histSeq_.push_back(std::move(newEntry));
    return newEntryReturn;
}

HistoryEntry *HistorySequence::insertEntry(long index,
        StateInfo *stateInfo, double discount,
        Action const &action, Observation const &obs,
        double immediateReward) {
    std::unique_ptr<HistoryEntry> newEntry = std::make_unique<HistoryEntry>(
                stateInfo, discount, this, histSeq_.size());
    newEntry->action_ = action;
    newEntry->observation_ = obs.copy();
    newEntry->immediateReward_ = immediateReward;
    HistoryEntry *newEntryReturn = newEntry.get();
    histSeq_.insert(histSeq_.begin() + index, std::move(newEntry));
    return newEntryReturn;
}

HistoryEntry *HistorySequence::getEntry(int entryId) const {
    return histSeq_[entryId].get();
}

std::vector<State const *> HistorySequence::getStates() const {
    std::vector<State const *> states;
    for (std::unique_ptr<HistoryEntry> const &entry : histSeq_) {
        states.push_back(entry->getState());
    }
    return states;
}

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
