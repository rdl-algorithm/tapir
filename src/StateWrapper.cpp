#include "StateWrapper.hpp"

#include <cmath>                        // for abs
#include <algorithm>                    // for find
#include <set>                          // for set
#include <vector>                       // for vector, vector<>::iterator
#include "ChangeType.hpp"               // for ChangeType
#include "State.hpp"                    // for State
class BeliefNode;
class HistoryEntry;

long StateWrapper::currId = 0;

StateWrapper::StateWrapper() :
    state(),
    id(0),
    usedInHistoryEntries(),
    usedInBeliefNodes(),
    chType(ChangeType::UNDEFINED) {
}

StateWrapper::StateWrapper(State &s) :
    StateWrapper() {
    this->state = s;
}

void StateWrapper::setId() {
    id = currId;
    currId++;
}

void StateWrapper::addInfo(HistoryEntry *h) {
    usedInHistoryEntries.push_back(h);
}

void StateWrapper::addInfo(BeliefNode *b) {
    usedInBeliefNodes.insert(b);
}

double StateWrapper::distL1(StateWrapper *st) {
    std::vector<double>::iterator it1, it2;
    double distUse = 0.0;
    for (it1 = state.vals.begin(), it2 = st->state.vals.begin();
            it1 != state.vals.end(); it1++, it2++) {
        distUse = distUse + std::abs(*it1 - *it2);
    }
    return distUse;
}

void StateWrapper::delUsedInHistEntry(HistoryEntry *toBeDeleted) {
    std::vector<HistoryEntry *>::iterator it = std::find(
                usedInHistoryEntries.begin(), usedInHistoryEntries.end(),
                toBeDeleted);
    (*it) = nullptr;
}
