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

long HistorySequence::currId = 0;

HistorySequence::HistorySequence() :
    HistorySequence(0) {
}

HistorySequence::HistorySequence(long startDepth) :
    id(currId),
    startDepth(startDepth),
    startAffectedIdx(LONG_MAX),
    endAffectedIdx(-1),
    histSeq(),
    changeType(ChangeType::UNDEFINED) {
    currId++;
}

HistorySequence::HistorySequence(std::unique_ptr<HistoryEntry> startEntry,
        long startDepth) :
    HistorySequence(startDepth) {
    startEntry->setSeqId(id);
    histSeq.push_back(std::move(startEntry));
}

// Do nothing!
HistorySequence::~HistorySequence() {
}

HistoryEntry *HistorySequence::getFirstEntry() {
    return histSeq.begin()->get();
}

HistoryEntry *HistorySequence::addEntry(Action const &action,
        Observation const &obs, StateInfo *stateInfo) {
    histSeq.back()->setNext(action, obs);
    std::unique_ptr<HistoryEntry> newEntry = std::make_unique<HistoryEntry>(
                stateInfo, id, histSeq.size());
    HistoryEntry *newEntryReturn = newEntry.get();
    histSeq.push_back(std::move(newEntry));
    return newEntryReturn;
}

HistoryEntry *HistorySequence::addEntry(StateInfo *stateInfo,
        Action const &action,
        Observation const &obs, double rew, double disc) {
    std::unique_ptr<HistoryEntry> newEntry = std::make_unique<HistoryEntry>(
                stateInfo, id, histSeq.size());
    newEntry->action = action;
    newEntry->obs = obs;
    newEntry->immediateReward = rew;
    newEntry->discount = disc;
    HistoryEntry *newEntryReturn = newEntry.get();
    histSeq.push_back(std::move(newEntry));
    return newEntryReturn;
}

HistoryEntry *HistorySequence::addEntry(StateInfo *stateInfo,
        Action const &action,
        Observation const &obs, double rew, double disc, long atIdx) {
    std::unique_ptr<HistoryEntry> newEntry = std::make_unique<HistoryEntry>(
                stateInfo, id, histSeq.size());
    newEntry->action = action;
    newEntry->obs = obs;
    newEntry->immediateReward = rew;
    newEntry->discount = disc;
    HistoryEntry *newEntryReturn = newEntry.get();
    histSeq.insert(histSeq.begin() + atIdx, std::move(newEntry));
    return newEntryReturn;
}

void HistorySequence::addEntry(std::unique_ptr<HistoryEntry> histEntry) {
    histSeq.push_back(std::move(histEntry));
}

HistoryEntry *HistorySequence::get(int entryId) {
    return histSeq[entryId].get();
}

std::vector<State const *> HistorySequence::getStates() {
    std::vector<State const *> states;
    for (std::unique_ptr<HistoryEntry> &entry : histSeq) {
        states.push_back(entry->stateInfo->getState());
    }
    return states;
}

void HistorySequence::fixEntryIds() {
    long i = 0;
    for (auto &entry : histSeq) {
        entry->entryId = i;
        i++;
    }
}
