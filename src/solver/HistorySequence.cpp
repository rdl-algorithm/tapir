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


void HistorySequence::registerStartingNode(BeliefNode *startNode) {
    HistoryEntry *firstEntry = getFirstEntry();
    if (firstEntry->isRegisteredAsParticle_) {
        firstEntry->associatedBeliefNode_->numberOfSequenceEdges_--;
    }
    firstEntry->registerNode(startNode);
    if (firstEntry->isRegisteredAsParticle_) {
        firstEntry->associatedBeliefNode_->numberOfSequenceEdges_++;
    }
}

void HistorySequence::registerRestOfSequence(bool registering,
        BeliefTree *policy) {
    bool isFirst = true;
    std::vector<std::unique_ptr<HistoryEntry>>::iterator historyIterator = histSeq_.begin();
    BeliefNode *currentNode = (*historyIterator)->associatedBeliefNode_;
    while (true) {
        HistoryEntry *entry = historyIterator->get();
        if (isFirst) {
            isFirst = false;
        } else {
            if (registering) {
                entry->registerNode(currentNode);
            } else {
                entry->registerNode(nullptr);
            }
        }
        if (entry->action_ == nullptr) {
            if (!isFirst) {
                if (registering) {
                    currentNode->numberOfSequenceEdges_++;
                } else {
                    currentNode->numberOfSequenceEdges_--;
                }
            }
            break;
        }
        if (registering) {
            currentNode = policy->createOrGetChild(currentNode,
                    *entry->action_, *entry->observation_);
            historyIterator++;
        } else {
            historyIterator++;
            currentNode = (*historyIterator)->associatedBeliefNode_;
        }
    }
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
