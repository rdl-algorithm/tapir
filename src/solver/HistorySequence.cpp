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


bool HistorySequence::testBackup(bool backingUp) {
    HistoryEntry *lastEntry = getLastEntry();
    if (lastEntry->getAction() != nullptr) {
        debug::show_message("ERROR: End of sequence has an action!?");
        return false;
    }

    for (std::unique_ptr<HistoryEntry> &entry : entrySequence_) {
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
    for (std::unique_ptr<HistoryEntry> &entry : entrySequence_) {
        entry->registerState(nullptr);
        if (entry->isRegisteredAsParticle()) {
            debug::show_message("ERROR: sequence should be fully deregistered"
                    " before deletion.");
        }
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

/* -------------- Registration methods ---------------- */
void HistorySequence::registerWith(BeliefNode *startNode, BeliefTree *policy) {
    bool registering = (startNode != nullptr);
    std::vector<std::unique_ptr<HistoryEntry>>::iterator historyIterator = entrySequence_.begin();
    if (registering && (*historyIterator)->isRegisteredAsParticle()) {
        debug::show_message("WARNING: Already registered this sequence!");
        registerWith(nullptr, policy);
        registerWith(startNode, policy);
    }
    BeliefNode *currentNode = startNode;
    for (; historyIterator != entrySequence_.end(); historyIterator++) {
        HistoryEntry *entry = historyIterator->get();
        if (registering) {
            entry->registerNode(currentNode);
            if (currentNode != nullptr && entry->action_ != nullptr) {
                // If a child already exists, we will use it.
                BeliefNode *nextNode = currentNode->getChild(
                        *entry->action_, *entry->observation_);
                if (nextNode == nullptr) {
                    // If no child exists, we check if one is desired.
                    HistoryEntry *nextEntry = (historyIterator+1)->get();
                    if (nextEntry->associatedBeliefNode_ != nullptr) {
                        nextNode = policy->createOrGetChild(currentNode,
                                    *entry->action_, *entry->observation_);
                    }
                }
                currentNode = nextNode;
            }
        } else {
            entry->registerNode(nullptr);
        }
    }
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
