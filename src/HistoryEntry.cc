#include <cstdlib>

#include "HistoryEntry.h"
#include "BeliefNode.h"

using namespace std;

HistoryEntry::HistoryEntry(State *st, unsigned long entryId) :
        st(st), entryId(entryId) {
    initialise();

}

HistoryEntry::HistoryEntry(State *st, unsigned long seqId, unsigned long entryId) :
        st(st), seqId(seqId), entryId(entryId) {
    initialise();
}

HistoryEntry::HistoryEntry(unsigned long seqId, unsigned long entryId, State *st,
        stringstream &sstr) :
        st(st), seqId(seqId), entryId(entryId) {
    initialise();
    string usrStr;
    sstr >> actId >> usrStr >> usrStr;
    obs.clear();
    while (usrStr.find(">") == string::npos) {
        obs.push_back(atof(usrStr.c_str()));
        sstr >> usrStr;
    }
    hasBeenBackup = true;
    sstr >> disc >> rew >> qVal;
}

HistoryEntry::~HistoryEntry() {
}

void HistoryEntry::initialise() {
    // Many of these values must be overridden later!
    actId = -1;
    hasBeenBackup = false;
    disc = 1.0;
    partOfBelNode = nullptr;
    rew = 0;
    qVal = 0;
}

void HistoryEntry::setBelNode(BeliefNode *bel) {
    partOfBelNode = bel;
}
/*
 void HistoryEntry::prepareDel() {
 if (actId != -1) {
 partOfBelNode->delParticle(this, actId, qVal);
 }
 }
 */
void HistoryEntry::write(ostream &os) {
//StateVals tmpVals; st->getVals(tmpVals);
//cout << "Entry ( " << tmpVals[0] << " " << tmpVals[1] << " ) ";
    os << "HistEntry < " << seqId << " " << entryId << " >: ( " << st->getId()
            << " " << actId << " < ";
    vector<double>::iterator itObs;
    for (itObs = obs.begin(); itObs != obs.end(); itObs++) {
        os << *itObs << " ";
    }
    os << " > " << disc << " " << rew << " " << qVal << " ) ";
    st->write(os);
}

void HistoryEntry::writeln(ostream &os) {
    os << "HistEntry < " << seqId << " " << entryId << " >: ( " << st->getId()
            << " " << actId << " < ";
    vector<double>::iterator itObs;
    for (itObs = obs.begin(); itObs != obs.end(); itObs++) {
        os << *itObs << " ";
    }
    os << " > " << disc << " " << rew << " " << qVal << " ) ";
    st->write(os);
    os << endl;
}

void HistoryEntry::writeSt(ostream &os) {
    os << "( ";
    st->write(os);
    os << " ) ";
}
