#ifndef STATEPOOL_H
#define STATEPOOL_H

#include <iostream>
#include <set>
#include <map>
#include "State.h"
#include "Model.h"

using namespace std;

struct CompStVals {
	bool operator()(const State* s1, const State* s2) const {
		StateVals s1Vals, s2Vals;
		s1->getVals(s1Vals); s2->getVals(s2Vals);
//cout << "s1Size: " << s1Vals.size() << " " << s2Vals.size() << endl;
//cout << "compare: ( " << s1Vals[0] << " " << s1Vals[1] << " ) ( " << s2Vals[0] << " " << s2Vals[1] << ")\n";		
		vector<double>::iterator it1, it2;
		for (it1 = s1Vals.begin(), it2 = s2Vals.begin(); it1 != s1Vals.end(); it1++, it2++) {
//cout << "it1: " << *it1 << " it2: " << *it2 << " min: " << *it1 - *it2 << endl;			
			if (*it1 - *it2 < 0) { return true; }
			else if (*it1 - *it2 > 0) { return false; }
		}
		return false;
	}
};

struct CompIdVals {
	bool operator()(const State* s1, const State* s2) const {
		if ((s1->getId() - s2->getId()) < 0) { return true; }
		else if ((s1->getId() - s2->getId()) > 0) { return false; }
		return false;
	}
};

class StatePool {
	public:
		friend class Solver;
		
		StatePool();
		~StatePool();
		
		void readStates(ifstream &inFile, Model *model);
		State* add(StateVals &sVals);
		State* getStPtr(long stId);
		void identifyAffectedStates(StateVals &lowLeft, StateVals &upRight, ChType chType, 
				set<State*> &affectedSt);
		//void getPosNNBelNode(State *s, double distTh, set<BeliefNode*> &res);
		void write (ostream &os);
		
	private:
		long nStates, nSDim;
		set<State*, CompStVals> allStates;
		vector<State*> allStatesIdx;
		//vector< multimap<double, set<State*, CompStVals>::iterator> > stStruct;
		vector< multimap<double, State*> > stStruct;
		
};
#endif
