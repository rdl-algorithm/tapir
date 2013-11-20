#include <sstream>
#include <algorithm>

#include <cmath>
#include <cstdlib>

#include "State.h"
#include "BeliefNode.h"
#include "HistoryEntry.h"

using namespace std;

long State::currId = 0;

State::State(StateVals &s) :
        s(s) {
    // Default values; should be overridden later.
    id = 0;
    chType = Change::UNDEFINED;
}

State::State(string &str, long nStVars) {
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

State::~State() {
    s.clear();
}

void State::setId() {
    id = currId;
    currId++;
}

void State::addInfo(HistoryEntry *h, BeliefNode *b) {
    usedInHistEntries.push_back(h);
    usedInBelNode.insert(b);
}

void State::addInfo(HistoryEntry *h) {
    usedInHistEntries.push_back(h);
}

void State::addInfo(BeliefNode *b) {
    usedInBelNode.insert(b);
}

double State::distL1(State *st) {
    vector<double>::iterator it1, it2;
    double distUse = 0.0;
    for (it1 = s.begin(), it2 = st->s.begin(); it1 != s.end(); it1++, it2++) {
        distUse = distUse + abs(*it1 - *it2);
    }
    return distUse;
}

void State::delUsedInHistEntry(HistoryEntry *toBeDeleted) {
    vector<HistoryEntry*>::iterator it = find(usedInHistEntries.begin(),
            usedInHistEntries.end(), toBeDeleted);
    (*it) = NULL;
}

void State::write(ostream &os) {
    os << "s " << id << " : ";
    vector<double>::iterator it;
    for (it = s.begin(); it != s.end(); it++) {
        os << *it << " ";
    }
}

void State::writeln(ostream &os) {
    os << "s " << id << " : ";
    vector<double>::iterator it;
    for (it = s.begin(); it != s.end(); it++) {
        os << *it << " ";
    }
    os << endl;
}
