#include <limits.h>
#include "HistorySeq.h"

long HistorySeq::currId = 0;

HistorySeq::HistorySeq(HistoryEntry *startEntry) {
	id = currId;
	startDepth = 0;
	startAffectedIdx = LONG_MAX;
	endAffectedIdx = -1;
	currId++;
	startEntry->setSeqId(id);
	histSeq.push_back(startEntry);
}

HistorySeq::HistorySeq(HistoryEntry *startEntry, long startDepth_): startDepth(startDepth_) {
	id = currId;
	startAffectedIdx = LONG_MAX;
	endAffectedIdx = -1;
	currId++;
	startEntry->setSeqId(id);
	histSeq.push_back(startEntry);
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

HistoryEntry* HistorySeq::addEntry(State *s, long actId, ObsVals &obs, double rew, double disc) {
	HistoryEntry* newEntry = new HistoryEntry(s, id, histSeq.size());
	newEntry->actId = actId;
	newEntry->obs = obs;
	newEntry->rew = rew;
	newEntry->disc = disc;
	histSeq.push_back(newEntry);
	return newEntry;
}

HistoryEntry* HistorySeq::addEntry(State *s, long actId, ObsVals &obs, double rew, double disc, long atIdx) {
	HistoryEntry* newEntry = new HistoryEntry(s, id, atIdx);
	newEntry->actId = actId;
	newEntry->obs = obs;
	newEntry->rew = rew;
	newEntry->disc = disc;
	histSeq.insert(histSeq.begin()+atIdx, newEntry);
	return newEntry;
}

void HistorySeq::addEntry(HistoryEntry *histEntry) {
	histSeq.push_back(histEntry);
}

/*
HistoryEntry* HistorySeq::addEntry(double discRew_, State *nxtSt) {
	histSeq.back()->discRew = discRew_;
	HistoryEntry* newEntry = new HistoryEntry(nxtSt, id, histSeq.size());
	histSeq.push_back(newEntry);
	return newEntry;
}
*/
/*
void HistorySeq::deleteAffectedEntries(Model *m) {
	vector<HistoryEntry*>::iterator itHistEntry = histSeq.begin()+startAffectedIdx-1;
	double prevQVal = (*itHistEntry)->qVal;
	(*itHistEntry)->rew = m->getReward((*itHistEntry)->st->s);
	(*itHistEntry)->qVal = (*itHistEntry)->disc*(*itHistEntry)->rew;
	(*itHistEntry)->partOfBelNode->updateVal(*itHistEntry, prevQVal, 0.0, false);
	(*itHistEntry)->actId = -1;
	itHistEntry ++;
	for (; itHistEntry != histSeq.end(); itHistEntry++) {
		(*itHistEntry)->partOfBelNode->delPartNUpdateVal(*itHistEntry, (*itHistEntry)->qVal, 0.0);
	}
	
	histSeq.erase(histSeq.begin()+startAffectedIdx, histSeq.end());
	cutAffected = true;
}
*/
/*
void HistorySeq::prepareDel() {
	vector<HistoryEntry*>::reverse_iterator it;
	for (it = histSeq.rbegin(); it != histSeq.rend(); it++) {
		(*it)->prepareDel();
		delete(*it);
		histSeq.erase(it);
	} 
}
*/

void HistorySeq::getStValsSeq(vector<StateVals> &seqStVals) {
	seqStVals.clear();
	vector<HistoryEntry*>::iterator it;
	for (it = histSeq.begin(); it != histSeq.end(); it++) {
		StateVals s; (*it)->st->getVals(s);
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
	long i = 0;
	vector<HistoryEntry*>::iterator it;
	for (it = histSeq.begin(); it != histSeq.end(); it++) {
		(*it)->writeln(os);
	}
}

void HistorySeq::writeln(ostream &os) {
	long i = 0;
	vector<HistoryEntry*>::iterator it;
	for (it = histSeq.begin(); it != histSeq.end(); it++) {
		(*it)->writeln(os);
	}
	os << endl;	
}
