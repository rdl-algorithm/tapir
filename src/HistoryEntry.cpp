#include "HistoryEntry.hpp"

#include <cstdlib>
using std::atof;

#include <ostream>
using std::endl;
using std::ostream;
#include <string>
using std::string;
#include <sstream>
using std::stringstream;
#include <vector>
using std::vector;

#include "StateWrapper.hpp"
#include "TextSerializer.hpp"

HistoryEntry::HistoryEntry(StateWrapper *st) :
            HistoryEntry(st, 0, 0) {
}

HistoryEntry::HistoryEntry(StateWrapper *st, long seqId, long entryId) :
            st(st),
            hasBeenBackup(false),
            seqId(seqId),
            entryId(entryId),
            disc(1.0),
            rew(0),
            qVal(0),
            actId(-1),
            obs(),
            partOfBelNode(nullptr) {
}

HistoryEntry::HistoryEntry(StateWrapper *st, long seqId, long entryId,
        stringstream &sstr) :
            HistoryEntry(st, seqId, entryId) {
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

void HistoryEntry::setBelNode(BeliefNode *bel) {
    partOfBelNode = bel;
}

void HistoryEntry::write(ostream &os) {
//State tmpVals; st->getVals(tmpVals);
//cout << "Entry ( " << tmpVals[0] << " " << tmpVals[1] << " ) ";
    os << "HistEntry < " << seqId << " " << entryId << " >: ( " << st->getId()
            << " " << actId << " < ";
    vector<double>::iterator itObs;
    for (itObs = obs.begin(); itObs != obs.end(); itObs++) {
        os << *itObs << " ";
    }
    os << " > " << disc << " " << rew << " " << qVal << " ) ";
    TextSerializer serializer;
    serializer.save(*st, os);
}

void HistoryEntry::writeln(ostream &os) {
    os << "HistEntry < " << seqId << " " << entryId << " >: ( " << st->getId()
            << " " << actId << " < ";
    vector<double>::iterator itObs;
    for (itObs = obs.begin(); itObs != obs.end(); itObs++) {
        os << *itObs << " ";
    }
    os << " > " << disc << " " << rew << " " << qVal << " ) ";
    TextSerializer serializer;
    serializer.save(*st, os);
    os << endl;
}

void HistoryEntry::writeSt(ostream &os) {
    os << "( ";
    TextSerializer serializer;
    serializer.save(*st, os);
    os << " ) ";
}
