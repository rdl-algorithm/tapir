#include <fstream>
#include <limits.h>
#include "Histories.h"

Histories::Histories() {
}

Histories::~Histories() {
	vector<HistorySeq*>::iterator it;
	for (it = allHistSeq.begin(); it != allHistSeq.end(); it++) {
		delete (*it);
	}
	allHistSeq.resize(0);
}

HistorySeq* Histories::addHistorySeq(State *s) {
	HistorySeq *newSeq = new HistorySeq(new HistoryEntry(s, 0));
	
	allHistSeq.push_back(newSeq);
	return newSeq;
}

HistorySeq* Histories::addHistorySeq(State *s, long startDepth) {
	HistorySeq *newSeq = new HistorySeq(new HistoryEntry(s, 0), startDepth);
	
	allHistSeq.push_back(newSeq);
	return newSeq;
}

void Histories::readHistories(ifstream &inFile, StatePool *stPool) {
	string tmpStr;
	getline(inFile, tmpStr);

	while(tmpStr.find("HISTORIES-BEGIN") == string::npos) {
		getline(inFile, tmpStr);
	}
	getline(inFile, tmpStr);	

	long seqId, entryId, stId;
	State *stPtr;
	pair< set<State*, CompStVals>::iterator, bool > ret;
	while(tmpStr.find("HISTORIES-END") == string::npos) {
		stringstream sstr(tmpStr);
		string usrStr;
		sstr >> usrStr >> usrStr >> seqId >> entryId >> usrStr >> usrStr >> stId;
		stPtr = stPool->getStPtr(stId);
		HistoryEntry* histEntry = new HistoryEntry(seqId, entryId, stPtr, sstr);
		stPtr->addInfo(histEntry);
		if (entryId == 0) {
			HistorySeq* histSeq = new HistorySeq(histEntry);
			allHistSeq.push_back(histSeq);
		}
		else {
			allHistSeq[seqId]->addEntry(histEntry);
		}
		getline(inFile, tmpStr);
	}
}
/*
void Histories::setAffected(long seqIdx, long entryIdx, ChType chType) {
	if (entryIdx < allHistSeq[seqIdx]->startAffectedIdx) {
		allHistSeq[seqIdx]->startAffectedIdx = entryIdx;
	}
	allHistSeq[seqIdx]->chType = max(allHistSeq[seqIdx]->chType, chType);
}

void Histories::resetAffected(set<long> &reachAffected, set<long> &notReachAffected) {
	set<long>::iterator itL;
	for (itL = reachAffected.begin(); itL != reachAffected.end(); itL++) {
		allHistSeq[*itL]->startAffectedIdx = LONG_MAX; 
	}
	for (itL = notReachAffected.begin(); itL != notReachAffected.end(); itL++) {
		allHistSeq[*itL]->startAffectedIdx = LONG_MAX;
	}
}

void Histories::resetAffected(set<long> &affected) {
	set<long>::iterator itL;
	for (itL = affected.begin(); itL != affected.end(); itL++) {
		allHistSeq[*itL]->startAffectedIdx = LONG_MAX; 
	}
}
*/
/*
void Histories::updateVal(double defaultVal, set<long> &reachAffectedHistSeq, set<long> &notReachAffectedHistSeq) {
	set<long>::iterator itL;
	for (itL = reachAffectedHistSeq.begin(); itL != reachAffectedHistSeq.end(); itL++) {
		allHistSeq[*itL]->updateVal(defaultVal);
	}
	for (itL = notReachAffectedHistSeq.begin(); itL != notReachAffectedHistSeq.end(); itL++) {
		allHistSeq[*itL]->updateVal(defaultVal);
	}
}

void Histories::eraseNUpdBel(BeliefNode *currNode, set<long> &affectedHistSeq) {
	set<long>::reverse_iterator itAffected;
	for (itAffected = affectedHistSeq.rbegin(); itAffected != affectedHistSeq.rend(); itAffected++) {
		allHistSeq[*itAffected]->prepareDel();
		delete(allHistSeq[*itAffected]);
		allHistSeq.erase(allHistSeq.begin()+(*itAffected));
	}
}
*/
void Histories::write(ostream &os) {
	long i = 0;
	vector<HistorySeq*>::iterator it;
//cerr << "#histSeq: " << allHistSeq.size() << endl;	
	for (it = allHistSeq.begin(); it != allHistSeq.end(); it++) {
		(*it)->write(os);
	}
}
