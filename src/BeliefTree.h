#ifndef BELIEFTREE_H
#define BELIEFTREE_H

#include <iostream>
#include "BeliefNode.h"
#include "Model.h"
#include "Histories.h"

using namespace std;

class BeliefTree {
	public:
		friend class Solver;
		
		BeliefTree();
		~BeliefTree();
		void deleteHelper(queue<BeliefNode*> &tmpNodes);
		
		void readPolicy(ifstream &inFile, Histories *allHist);
		void write(ostream &os);
		void writeHelp(queue<BeliefNode*> &tmpNodes, ostream &os);
		
		BeliefNode* getRoot() { return root; }
		
	private:
		BeliefNode *root;
		vector<BeliefNode*> allNodes;
		
		
};
#endif
