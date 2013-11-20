#include <climits>

#include "HistorySeq.h"

using namespace std;

long HistorySeq::currId = 0;

HistorySeq::HistorySeq(HistoryEntry *startEntry) :
        HistorySeq(startEntry, 0) {
}

HistorySeq::HistorySeq(HistoryEntry *startEntry, long startDepth) :
        startDepth(startDepth) {
    id = currId;
    currId++;
    startAffectedIdx = LONG_MAX;
    endAffectedIdx = -1;
    startEntry->setSeqId(id);
    histSeq.push_back(startEntry);

    chType = Change::UNDEFINED;
}

HistorySeq::~HistorySeq() {
    vector<HistoryEntry*>::iterator it;
    for (it = histSeq.begin(); it != histSeq.end(); it++) {
        delete (*it);
    }
    histSeq.resize(0);
}

HistoryEntry* HistorySeq::getFirstEntry() {
    return histSeq[0];
}

HistoryEntry* HistorySeq::addEntry(long actId, ObsVals &obs, State *s) {
    histSeq.back()->setNxt(actId, obs);
    HistoryEntry* newEntry = new HistoryEntry(s, id, histSeq.size());
    histSeq.push_back(newEntry);
    return newEntry;
}

HistoryEntry* HistorySeq::addEntry(State *s, long actId, ObsVals &obs,
        double rew, double disc) {
    HistoryEntry* newEntry = new HistoryEntry(s, id, histSeq.size());
    newEntry->actId = actId;
    newEntry->obs = obs;
    newEntry->rew = rew;
    newEntry->disc = disc;
    histSeq.push_back(newEntry);
    return newEntry;
}

HistoryEntry* HistorySeq::addEntry(State *s, long actId, ObsVals &obs,
        double rew, double disc, long atIdx) {
    HistoryEntry* newEntry = new HistoryEntry(s, id, atIdx);
    newEntry->actId = actId;
    newEntry->obs = obs;
    newEntry->rew = rew;
    newEntry->disc = disc;
    histSeq.insert(histSeq.begin() + atIdx, newEntry);
    return newEntry;
}

void HistorySeq::addEntry(HistoryEntry *histEntry) {
    histSeq.push_back(histEntry);
}

void HistorySeq::getStValsSeq(vector<StateVals> &seqStVals) {
    seqStVals.clear();
    vector<HistoryEntry*>::iterator it;
    for (it = histSeq.begin(); it != histSeq.end(); it++) {
        StateVals s;
        (*it)->st->getVals(s);
        seqStVals.push_back(s);
    }
}

void HistorySeq::fixEntryId() {
    vector<HistoryEntry*>::iterator itH;
    long i = 0;
    for (itH = histSeq.begin(); itH != histSeq.end(); itH++, i++) {
        (*itH)->entryId = i;
    }
}

void HistorySeq::write(ostream &os) {
    vector<HistoryEntry*>::iterator it;
    for (it = histSeq.begin(); it != histSeq.end(); it++) {
        (*it)->writeln(os);
    }
}

void HistorySeq::writeln(ostream &os) {
    vector<HistoryEntry*>::iterator it;
    for (it = histSeq.begin(); it != histSeq.end(); it++) {
        (*it)->writeln(os);
    }
    os << endl;
}
