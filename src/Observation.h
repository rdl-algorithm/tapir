#ifndef OBSERVATION_H
#define OBSERVATION_H

#include <iostream>
#include <vector>
#include <queue>
#include "Model.h"

using namespace std;

class BeliefNode;

class Observation {
	public:
		Observation();
		Observation(ObsVals &o, BeliefNode* nxtBelNode);
		~Observation();
		
		bool isObs(ObsVals &o);
		BeliefNode* getNodeChild();
		void getChildren(queue<BeliefNode*> &res);
		void write(ostream &os);
		void writeNGetChildren(ostream &os, queue<BeliefNode*> &res);
	
	private:
		ObsVals vals;
		BeliefNode* child;
		
};
#endif
