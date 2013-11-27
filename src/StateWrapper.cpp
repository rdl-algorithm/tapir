#include "StateWrapper.hpp"

#include <cmath>
using std::abs;

#include <algorithm>
using std::find;
#include <iostream>
using std::endl;
using std::ostream;
#include <set>
using std::set;
#include <sstream>
using std::stringstream;
#include <string>
using std::string;
#include <vector>
using std::vector;

long StateWrapper::currId = 0;

StateWrapper::StateWrapper() :
            s(),
            id(0),
            usedInHistoryEntries(),
            usedInBeliefNodes(),
            chType(ChangeType::UNDEFINED) {
}

StateWrapper::StateWrapper(State &s) :
            StateWrapper() {
    this->s = s;
}

StateWrapper::StateWrapper(string &str, long nStVars) :
            StateWrapper() {
    // Load info from the string.
    stringstream sstr(str);
    string tmpStr;
    sstr >> tmpStr >> id >> tmpStr;
    for (long i = 0; i < nStVars; i++) {
        double tmpDouble;
        sstr >> tmpDouble;
        s.push_back(tmpDouble);
    }
    if (id > currId) {
        currId = id + 1;
    }
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
    for (it1 = s.begin(), it2 = st->s.begin(); it1 != s.end(); it1++, it2++) {
        distUse = distUse + abs(*it1 - *it2);
    }
    return distUse;
}

void StateWrapper::delUsedInHistEntry(HistoryEntry *toBeDeleted) {
    vector<HistoryEntry*>::iterator it = find(usedInHistoryEntries.begin(),
            usedInHistoryEntries.end(), toBeDeleted);
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
