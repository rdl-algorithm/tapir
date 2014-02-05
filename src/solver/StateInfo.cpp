#include "StateInfo.hpp"

#include <algorithm>                    // for find
#include <memory>                       // for unique_ptr
#include <set>                          // for set
#include <utility>                      // for move
#include <vector>                       // for vector, vector<>::iterator

#include "ChangeFlags.hpp"               // for ChangeFlags, ChangeFlags::UNCHANGED
#include "State.hpp"                    // for State

namespace solver {
class BeliefNode;
class HistoryEntry;

long StateInfo::currId = 0;

// Private constructor for serialization
StateInfo::StateInfo() :
    state_(nullptr),
    id_(0),
    usedInHistoryEntries_(),
    changeFlags_(ChangeFlags::UNCHANGED) {
}

StateInfo::StateInfo(std::unique_ptr<State> state) :
    StateInfo() {
    this->state_ = std::move(state);
}

// Do nothing!
StateInfo::~StateInfo() {
}

long StateInfo::getId() {
    return id_;
}

void StateInfo::setId() {
    id_ = currId;
    currId++;
}

void StateInfo::addHistoryEntry(HistoryEntry *entry) {
    usedInHistoryEntries_.insert(entry);
}

void StateInfo::removeHistoryEntry(HistoryEntry *entry) {
    usedInHistoryEntries_.erase(entry);
}

void StateInfo::resetChangeFlags() {
    changeFlags_ = ChangeFlags::UNCHANGED;
}

void StateInfo::setChangeFlags(ChangeFlags flags) {
    changeFlags_ |= flags;
}

} /* namespace solver */
