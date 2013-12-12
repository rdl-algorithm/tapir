#include "HistorySequence.hpp"

#include <climits>                      // for LONG_MAX

#include <memory>                       // for unique_ptr
#include <utility>                      // for move
#include <vector>                       // for vector, __alloc_traits<>::value_type

#include "defs.hpp"                     // for make_unique

#include "Action.hpp"                   // for Action
#include "ChangeType.hpp"               // for ChangeType, ChangeType::UNDEFINED
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
    startAffectedIdx_(LONG_MAX),
    endAffectedIdx_(-1),
    changeType_(ChangeType::UNDEFINED) {
    currId++;
}

HistorySequence::HistorySequence(std::unique_ptr<HistoryEntry> startEntry,
        long startDepth) :
    HistorySequence(startDepth) {
    startEntry->seqId_ = id_;
    histSeq_.push_back(std::move(startEntry));
}

// Do nothing!
HistorySequence::~HistorySequence() {
}

HistoryEntry *HistorySequence::getFirstEntry() {
    return histSeq_.begin()->get();
}

HistoryEntry *HistorySequence::addEntry(StateInfo *stateInfo) {
    std::unique_ptr<HistoryEntry> newEntry = std::make_unique<HistoryEntry>(
                stateInfo, id_, histSeq_.size());
    HistoryEntry *newEntryReturn = newEntry.get();
    histSeq_.push_back(std::move(newEntry));
    return newEntryReturn;
}

HistoryEntry *HistorySequence::addEntry(StateInfo *stateInfo,
        Action const &action,
        Observation const &obs, double rew, double disc) {
    std::unique_ptr<HistoryEntry> newEntry = std::make_unique<HistoryEntry>(
                stateInfo, id_, histSeq_.size());
    newEntry->action_ = action;
    newEntry->observation_ = obs;
    newEntry->immediateReward_ = rew;
    newEntry->discount_ = disc;
    HistoryEntry *newEntryReturn = newEntry.get();
    histSeq_.push_back(std::move(newEntry));
    return newEntryReturn;
}

HistoryEntry *HistorySequence::addEntry(StateInfo *stateInfo,
        Action const &action,
        Observation const &obs, double rew, double disc, long atIdx) {
    std::unique_ptr<HistoryEntry> newEntry = std::make_unique<HistoryEntry>(
                stateInfo, id_, histSeq_.size());
    newEntry->action_ = action;
    newEntry->observation_ = obs;
    newEntry->immediateReward_ = rew;
    newEntry->discount_ = disc;
    HistoryEntry *newEntryReturn = newEntry.get();
    histSeq_.insert(histSeq_.begin() + atIdx, std::move(newEntry));
    return newEntryReturn;
}

void HistorySequence::addEntry(std::unique_ptr<HistoryEntry> histEntry) {
    histSeq_.push_back(std::move(histEntry));
}

HistoryEntry *HistorySequence::get(int entryId) {
    return histSeq_[entryId].get();
}

std::vector<State const *> HistorySequence::getStates() {
    std::vector<State const *> states;
    for (std::unique_ptr<HistoryEntry> &entry : histSeq_) {
        states.push_back(entry->stateInfo_->getState());
    }
    return states;
}

void HistorySequence::fixEntryIds() {
    long i = 0;
    for (auto &entry : histSeq_) {
        entry->entryId_ = i;
        i++;
    }
}
} /* namespace solver */
