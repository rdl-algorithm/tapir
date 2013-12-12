#include "StateInfo.hpp"

#include <algorithm>                    // for find
#include <memory>                       // for unique_ptr
#include <set>                          // for set
#include <utility>                      // for move
#include <vector>                       // for vector, vector<>::iterator

#include "ChangeType.hpp"               // for ChangeType, ChangeType::UNDEFINED
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
    usedInBeliefNodes_(),
    changeType_(ChangeType::UNDEFINED) {
}

StateInfo::StateInfo(std::unique_ptr<State> state) :
    StateInfo() {
    this->state_ = std::move(state);
}

// Do nothing!
StateInfo::~StateInfo() {
}

void StateInfo::setId() {
    id_ = currId;
    currId++;
}

void StateInfo::addHistoryEntry(HistoryEntry *h) {
    usedInHistoryEntries_.push_back(h);
}

void StateInfo::addBeliefNode(BeliefNode *b) {
    usedInBeliefNodes_.insert(b);
}

void StateInfo::delUsedInHistEntry(HistoryEntry *toBeDeleted) {
    std::vector<HistoryEntry *>::iterator it = std::find(
                usedInHistoryEntries_.begin(), usedInHistoryEntries_.end(),
                toBeDeleted);
    (*it) = nullptr;
}
} /* namespace solver */
