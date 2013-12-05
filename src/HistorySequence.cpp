#include "HistorySequence.hpp"

#include <climits>                      // for LONG_MAX
#include <vector>                       // for vector, vector<>::iterator
#include "ChangeType.hpp"               // for ChangeType
#include "HistoryEntry.hpp"             // for HistoryEntry
#include "Observation.hpp"              // for Observation
#include "State.hpp"                    // for State
#include "StateWrapper.hpp"             // for StateWrapper
long HistorySequence::currId = 0;

HistorySequence::HistorySequence() :
    HistorySequence(0) {
}

HistorySequence::HistorySequence(long startDepth) :
    id(currId),
    startDepth(startDepth),
    startAffectedIdx(LONG_MAX),
    endAffectedIdx(-1),
    histSeq(),
    chType(ChangeType::UNDEFINED) {
    currId++;
}

HistorySequence::HistorySequence(HistoryEntry *startEntry, long startDepth) :
    HistorySequence(startDepth) {
    startEntry->setSeqId(id);
    histSeq.push_back(startEntry);
}

HistorySequence::~HistorySequence() {
    std::vector<HistoryEntry *>::iterator it;
    for (it = histSeq.begin(); it != histSeq.end(); it++) {
        delete (*it);
    }
    histSeq.resize(0);
}

HistoryEntry *HistorySequence::getFirstEntry() {
    return histSeq[0];
}

HistoryEntry *HistorySequence::addEntry(long actId, Observation &obs,
                                        StateWrapper *s) {
    histSeq.back()->setNxt(actId, obs);
    HistoryEntry *newEntry = new HistoryEntry(s, id, histSeq.size());
    histSeq.push_back(newEntry);
    return newEntry;
}

HistoryEntry *HistorySequence::addEntry(StateWrapper *s, long actId,
                                        Observation &obs, double rew, double disc) {
    HistoryEntry *newEntry = new HistoryEntry(s, id, histSeq.size());
    newEntry->actId = actId;
    newEntry->obs = obs;
    newEntry->rew = rew;
    newEntry->disc = disc;
    histSeq.push_back(newEntry);
    return newEntry;
}

HistoryEntry *HistorySequence::addEntry(StateWrapper *s, long actId,
                                        Observation &obs, double rew, double disc, long atIdx) {
    HistoryEntry *newEntry = new HistoryEntry(s, id, atIdx);
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

void HistorySequence::getStValsSeq(std::vector<State> &seqStVals) {
    seqStVals.clear();
    std::vector<HistoryEntry *>::iterator it;
    for (it = histSeq.begin(); it != histSeq.end(); it++) {
        State s;
        (*it)->st->getVals(s);
        seqStVals.push_back(s);
    }
}

void HistorySequence::fixEntryId() {
    std::vector<HistoryEntry *>::iterator itH;
    long i = 0;
    for (itH = histSeq.begin(); itH != histSeq.end(); itH++, i++) {
        (*itH)->entryId = i;
    }
}
