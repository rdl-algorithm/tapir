#ifndef STATE_H
#define STATE_H

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include "Model.h"

using namespace std;

class BeliefNode;
class HistoryEntry;

class State {
	public:		
		friend class StatePool;
		friend class Solver;

		double distUse;
		
		State(StateVals &s_);
		State(string &str, long nStVars);
		~State();

		void setId();
		void addInfo(HistoryEntry *h, BeliefNode *b);
		void addInfo(HistoryEntry *h);
		void addInfo(BeliefNode *b);
		double distL1(State *st);

		void delUsedInHistEntry(HistoryEntry *toBeDeleted);
		
		void write(ostream &os);
		void writeln(ostream &os);

		void getVals(StateVals &res) const { res = s; }
		long getId() const { return id; }
		
	private:
		static long currId;

		long id;
		StateVals s;
		vector<HistoryEntry*> usedInHistEntries;
		set<BeliefNode*> usedInBelNode;
		
		ChType chType;
};
#endif
