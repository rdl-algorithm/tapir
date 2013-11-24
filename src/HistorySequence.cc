#include <climits>

#include "HistorySequence.h"

using namespace std;

long HistorySequence::currId = 0;

HistorySequence::HistorySequence(HistoryEntry *startEntry) :
        HistorySequence(startEntry, 0) {
}

HistorySequence::HistorySequence(HistoryEntry *startEntry, long startDepth) :
        startDepth(startDepth) {
    id = currId;
    currId++;
    startAffectedIdx = LONG_MAX;
    endAffectedIdx = -1;
    startEntry->setSeqId(id);
    histSeq.push_back(startEntry);

    chType = Change::UNDEFINED;
}

HistorySequence::~HistorySequence() {
    vector<HistoryEntry*>::iterator it;
    for (it = histSeq.begin(); it != histSeq.end(); it++) {
        delete (*it);
    }
    histSeq.resize(0);
}

HistoryEntry* HistorySequence::getFirstEntry() {
    return histSeq[0];
}

HistoryEntry* HistorySequence::addEntry(long actId, ObsVals &obs,
        StateWrapper *s) {
    histSeq.back()->setNxt(actId, obs);
    HistoryEntry* newEntry = new HistoryEntry(s, id, histSeq.size());
    histSeq.push_back(newEntry);
    return newEntry;
}

HistoryEntry* HistorySequence::addEntry(StateWrapper *s, long actId,
        ObsVals &obs, double rew, double disc) {
    HistoryEntry* newEntry = new HistoryEntry(s, id, histSeq.size());
    newEntry->actId = actId;
    newEntry->obs = obs;
    newEntry->rew = rew;
    newEntry->disc = disc;
    histSeq.push_back(newEntry);
    return newEntry;
}

HistoryEntry* HistorySequence::addEntry(StateWrapper *s, long actId,
        ObsVals &obs, double rew, double disc, long atIdx) {
    HistoryEntry* newEntry = new HistoryEntry(s, id, atIdx);
    newEntry->actId = actId;
    newEntry->obs = obs;
    newEntry->rew = rew;
    newEntry->disc = disc;
    histSeq.insert(histSeq.begin() + atIdx, newEntry);
    return newEntry;
}

void HistorySequence::addEntry(HistoryEntry *histEntry) {
    histSeq.push_back(histEntry);
}

void HistorySequence::getStValsSeq(vector<StateVals> &seqStVals) {
    seqStVals.clear();
    vector<HistoryEntry*>::iterator it;
    for (it = histSeq.begin(); it != histSeq.end(); it++) {
        StateVals s;
        (*it)->st->getVals(s);
        seqStVals.push_back(s);
    }
}

void HistorySequence::fixEntryId() {
    vector<HistoryEntry*>::iterator itH;
    long i = 0;
    for (itH = histSeq.begin(); itH != histSeq.end(); itH++, i++) {
        (*itH)->entryId = i;
    }
}

void HistorySequence::write(ostream &os) {
    vector<HistoryEntry*>::iterator it;
    for (it = histSeq.begin(); it != histSeq.end(); it++) {
        (*it)->writeln(os);
    }
}

void HistorySequence::writeln(ostream &os) {
    vector<HistoryEntry*>::iterator it;
    for (it = histSeq.begin(); it != histSeq.end(); it++) {
        (*it)->writeln(os);
    }
    os << endl;
}
