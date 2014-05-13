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
    HistorySequence(-1) {
}

HistorySequence::HistorySequence(long id) :
    id_(id),
    entrySequence_(),
    startAffectedIdx_(std::numeric_limits<long>::max()),
    endAffectedIdx_(-1),
    changeFlags_(ChangeFlags::UNCHANGED) {
}

// Do nothing!
HistorySequence::~HistorySequence() {
}

/* ------------------ Simple getters ------------------- */
long HistorySequence::getId() const {
    return id_;
}
long HistorySequence::getLength() const {
    return entrySequence_.size();
}
HistoryEntry *HistorySequence::getEntry(long entryId) const {
    return entrySequence_[entryId].get();
}
HistoryEntry *HistorySequence::getFirstEntry() const {
    return entrySequence_.begin()->get();
}
HistoryEntry *HistorySequence::getLastEntry() const {
    return entrySequence_.rbegin()->get();
}
std::vector<State const *> HistorySequence::getStates() const {
    std::vector<State const *> states;
    for (std::unique_ptr<HistoryEntry> const &entry : entrySequence_) {
        states.push_back(entry->getState());
    }
    return states;
}


/* ============================ PRIVATE ============================ */


/* ----------- Methods to add or remove history entries ------------- */
void HistorySequence::reset() {
    for (std::unique_ptr<HistoryEntry> &entry : entrySequence_) {
        entry->registerNode(nullptr);
        entry->registerState(nullptr);
    }
    entrySequence_.clear();
}

HistoryEntry *HistorySequence::addEntry(StateInfo *stateInfo) {
    std::unique_ptr<HistoryEntry> newEntry = std::make_unique<HistoryEntry>(
                stateInfo, this, entrySequence_.size());
    HistoryEntry *newEntryReturn = newEntry.get();
    entrySequence_.push_back(std::move(newEntry));
    return newEntryReturn;
}

/* -------------- Change flagging methods ---------------- */
void HistorySequence::resetChangeFlags() {
    changeFlags_ = ChangeFlags::UNCHANGED;
    for (std::unique_ptr<HistoryEntry> &entry : entrySequence_) {
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
