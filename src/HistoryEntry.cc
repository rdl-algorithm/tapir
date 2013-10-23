#include <stdlib.h>
#include "HistoryEntry.h"
#include "BeliefNode.h"

HistoryEntry::HistoryEntry(State *st_, long entryId_): st(st_), entryId(entryId_) {
	actId = -1;
	hasBeenBackup = false;
}

HistoryEntry::HistoryEntry(State *st_, long seqId_, long entryId_): st(st_), seqId(seqId_), entryId(entryId_) {
	actId = -1;
	hasBeenBackup = false;
}

HistoryEntry::HistoryEntry(long seqId_, long entryId_, State *st_, stringstream &sstr): st(st_), seqId(seqId_), entryId(entryId_) {
	string usrStr;
	sstr >> actId >> usrStr >> usrStr;
	obs.clear();
	while (usrStr.find(">") == string::npos) {
		obs.push_back(atof(usrStr.c_str()));
		sstr >> usrStr;
	}
	sstr >> disc >> rew >> qVal;
	hasBeenBackup = true;
}

HistoryEntry::~HistoryEntry() {
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
	os << "HistEntry < " << seqId << " " << entryId << " >: ( " << st->getId() << " " << actId << " < ";
	vector<double>::iterator itObs;
	for (itObs = obs.begin(); itObs != obs.end(); itObs++) {
		os << *itObs << " ";
	}
	os << " > " << disc << " " << rew << " " << qVal << " ) ";
	st->write(os);	
} 

void HistoryEntry::writeln(ostream &os) {
	os << "HistEntry < " << seqId << " " << entryId << " >: ( " << st->getId() << " " << actId << " < ";
	vector<double>::iterator itObs;
	for (itObs = obs.begin(); itObs != obs.end(); itObs++) {
		os << *itObs << " ";
	}
	os << " > " << disc << " " << rew << " " << qVal << " ) ";
	st->write(os);	
	os << endl;
} 

void HistoryEntry::writeSt(ostream &os) {
	os << "( "; st->write(os); os << " ) "; 
}
