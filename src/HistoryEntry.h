#ifndef HISTORYENTRY_H
#define HISTORYENTRY_H

#include <iostream>
#include <string>
#include <sstream>
#include "State.h"
#include "Model.h"

using namespace std;

class BeliefNode;

class HistoryEntry {
	public:
		friend class HistorySeq;
		friend class BeliefNode;
		friend class Solver;
		
		HistoryEntry(State *st_, long entryId_);
		HistoryEntry(State *st_, long seqId_, long entryIdId_);
		HistoryEntry(long seqId_, long entryId_, State *st_, stringstream &sstr);
		~HistoryEntry();
		
		void setBelNode(BeliefNode *bel);
		//void prepareDel();
		
		void write(ostream &os);
		void writeln(ostream &os);
		void writeSt(ostream &os);
		
		inline void setSeqId(long seqId_) { seqId = seqId_; }
		inline void setNxt(long actId_, ObsVals &obs_) { actId = actId_, obs = obs_; }
		inline BeliefNode* getPartOfBelNode() { return partOfBelNode; }
		inline long getId() { return entryId; }
		inline long getSeqId() { return seqId; }
		inline long getActId() { return actId; }
		
	private:
		bool hasBeenBackup;
		long entryId, seqId;
		State *st;
		double disc, rew, qVal;	// disc: discount factor for the immediate reward, rew: not discounted immediate reward, qVal: discounted total reward.
		long actId;
		ObsVals obs;

		BeliefNode* partOfBelNode;		
		
};
#endif
