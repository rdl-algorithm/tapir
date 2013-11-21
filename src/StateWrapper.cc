#include <sstream>
#include <algorithm>

#include <cmath>
#include <cstdlib>

#include "StateWrapper.h"
#include "BeliefNode.h"
#include "HistoryEntry.h"

using namespace std;

long StateWrapper::currId = 0;

StateWrapper::StateWrapper(StateVals &s) :
        s(s) {
    // Default values; should be overridden later.
    id = 0;
    chType = Change::UNDEFINED;
}

StateWrapper::StateWrapper(string &str, long nStVars) {
    // Default values; should be overridden later.
    id = 0;
    chType = Change::UNDEFINED;

    // Load info from the string.
    s.clear();
    stringstream sstr(str);
    string tmpStr;
    double tmpDouble = -1;
    sstr >> tmpStr >> id >> tmpStr;
    for (long i = 0; i < nStVars; i++) {
        sstr >> tmpDouble;
        s.push_back(tmpDouble);
    }
    if (id > currId) {
        currId = id + 1;
    }
    chType = Change::UNDEFINED;
}

StateWrapper::~StateWrapper() {
    s.clear();
}

void StateWrapper::setId() {
    id = currId;
    currId++;
}

void StateWrapper::addInfo(HistoryEntry *h, BeliefNode *b) {
    usedInHistEntries.push_back(h);
    usedInBelNode.insert(b);
}

void StateWrapper::addInfo(HistoryEntry *h) {
    usedInHistEntries.push_back(h);
}

void StateWrapper::addInfo(BeliefNode *b) {
    usedInBelNode.insert(b);
}

double StateWrapper::distL1(StateWrapper *st) {
    vector<double>::iterator it1, it2;
    double distUse = 0.0;
    for (it1 = s.begin(), it2 = st->s.begin(); it1 != s.end(); it1++, it2++) {
        distUse = distUse + abs(*it1 - *it2);
    }
    return distUse;
}

void StateWrapper::delUsedInHistEntry(HistoryEntry *toBeDeleted) {
    vector<HistoryEntry*>::iterator it = find(usedInHistEntries.begin(),
            usedInHistEntries.end(), toBeDeleted);
    (*it) = nullptr;
}

void StateWrapper::write(ostream &os) {
    os << "s " << id << " : ";
    vector<double>::iterator it;
    for (it = s.begin(); it != s.end(); it++) {
        os << *it << " ";
    }
}

void StateWrapper::writeln(ostream &os) {
    os << "s " << id << " : ";
    vector<double>::iterator it;
    for (it = s.begin(); it != s.end(); it++) {
        os << *it << " ";
    }
    os << endl;
}
