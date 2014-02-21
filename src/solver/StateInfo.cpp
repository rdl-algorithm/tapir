#include "StateInfo.hpp"

#include <algorithm>                    // for find
#include <memory>                       // for unique_ptr
#include <set>                          // for set
#include <utility>                      // for move
#include <vector>                       // for vector, vector<>::iterator

#include "ChangeFlags.hpp"              // for ChangeFlags, ChangeFlags::UNCHANGED
#include "topology/State.hpp"                    // for State

namespace solver {
class BeliefNode;
class HistoryEntry;

long StateInfo::currId = 0;

StateInfo::StateInfo(std::unique_ptr<State> state) :
    state_(std::move(state)),
    id_(0),
    usedInHistoryEntries_(),
    changeFlags_(ChangeFlags::UNCHANGED) {
}

// Private constructor for serialization
StateInfo::StateInfo() :
    StateInfo(nullptr) {
}

StateInfo::StateInfo(State const &state) :
        StateInfo(state.copy()) {
}

// Do nothing!
StateInfo::~StateInfo() {
}

long StateInfo::getId() const {
    return id_;
}

void StateInfo::setId() {
    id_ = currId;
    currId++;
}

State const *StateInfo::getState() const {
    return state_.get();
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
