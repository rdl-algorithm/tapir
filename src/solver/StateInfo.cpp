/** @file StateInfo.cpp
 *
 * Contains the implementation of the StateInfo class.
 */
#include "solver/StateInfo.hpp"

#include <algorithm>                    // for find
#include <memory>                       // for unique_ptr
#include <set>                          // for set
#include <utility>                      // for move
#include <vector>                       // for vector, vector<>::iterator

#include "solver/abstract-problem/State.hpp"                    // for State
#include "solver/changes/ChangeFlags.hpp"              // for ChangeFlags, ChangeFlags::UNCHANGED

namespace solver {
class BeliefNode;
class HistoryEntry;

StateInfo::StateInfo(std::unique_ptr<State> state) :
    state_(std::move(state)),
    id_(-1),
    usedInHistoryEntries_(),
    changeFlags_(ChangeFlags::UNCHANGED) {
}

// Constructor for serialization.
StateInfo::StateInfo() :
    StateInfo(nullptr) {
}

StateInfo::StateInfo(State const &state) :
        StateInfo(state.copy()) {
}

// Do nothing!
StateInfo::~StateInfo() {
}


/* ---------------------- Simple getters  ---------------------- */
long StateInfo::getId() const {
    return id_;
}
State const *StateInfo::getState() const {
    return state_.get();
}


/* ============================ PRIVATE ============================ */


/* ----------------- History entry registration  ----------------- */
void StateInfo::addHistoryEntry(HistoryEntry *entry) {
    usedInHistoryEntries_.insert(entry);
}
void StateInfo::removeHistoryEntry(HistoryEntry *entry) {
    usedInHistoryEntries_.erase(entry);
}

/* ---------------------- Model change handling  ---------------------- */
void StateInfo::resetChangeFlags() {
    changeFlags_ = ChangeFlags::UNCHANGED;
}

void StateInfo::setChangeFlags(ChangeFlags flags) {
    changeFlags_ |= flags;
}

} /* namespace solver */
