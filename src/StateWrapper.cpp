#include "StateWrapper.hpp"

#include <cmath>
using std::abs;

#include <algorithm>
using std::find;
#include <set>
using std::set;
#include <vector>
using std::vector;

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
    vector<double>::iterator it1, it2;
    double distUse = 0.0;
    for (it1 = state.vals.begin(), it2 = st->state.vals.begin();
            it1 != state.vals.end(); it1++, it2++) {
        distUse = distUse + abs(*it1 - *it2);
    }
    return distUse;
}

void StateWrapper::delUsedInHistEntry(HistoryEntry *toBeDeleted) {
    vector<HistoryEntry*>::iterator it = find(usedInHistoryEntries.begin(),
            usedInHistoryEntries.end(), toBeDeleted);
    (*it) = nullptr;
}
