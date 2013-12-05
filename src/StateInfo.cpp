#include "StateInfo.hpp"

#include <cmath>                        // for abs

#include <algorithm>                    // for find
#include <memory>                       // for unique_ptr
#include <set>                          // for set
#include <utility>                      // for move
#include <vector>                       // for vector, vector<>::iterator

#include "ChangeType.hpp"               // for ChangeType
#include "State.hpp"                    // for State

class BeliefNode;
class HistoryEntry;

long StateInfo::currId = 0;

// Private constructor for serialization
StateInfo::StateInfo() :
    state(nullptr),
    id(0),
    usedInHistoryEntries(),
    usedInBeliefNodes(),
    chType(ChangeType::UNDEFINED) {
}

StateInfo::StateInfo(std::unique_ptr<State> state) :
    StateInfo() {
    this->state = std::move(state);
}

void StateInfo::setId() {
    id = currId;
    currId++;
}

void StateInfo::addHistoryEntry(HistoryEntry *h) {
    usedInHistoryEntries.push_back(h);
}

void StateInfo::addBeliefNode(BeliefNode *b) {
    usedInBeliefNodes.insert(b);
}

void StateInfo::delUsedInHistEntry(HistoryEntry *toBeDeleted) {
    std::vector<HistoryEntry *>::iterator it = std::find(
                usedInHistoryEntries.begin(), usedInHistoryEntries.end(),
                toBeDeleted);
    (*it) = nullptr;
}
