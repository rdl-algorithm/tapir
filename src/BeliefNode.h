#ifndef BELIEFNODE_H
#define BELIEFNODE_H

#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <time.h>
//#include <cv.h>
//#include <highgui.h>
#include "Histories.h"
#include "HistoryEntry.h"
#include "Action.h"

using namespace std;
//using namespace cv;

class BeliefNode {
	public:
		friend class BeliefTree;
		friend class Solver;
		
		bool distChecked;
		static long maxParticles;
		static long nStVars;

		
		BeliefNode();
		BeliefNode(long id_);
		~BeliefNode();
	
		void set(stringstream &sstr, Histories *allHist);
		void setAct(string str, vector<BeliefNode*> &tmpNodes);
		long getUCBAct();
		long getBestAct();
		void add(HistoryEntry *newHistEntry);
		BeliefNode* addChild(long actIdx, ObsVals &obs, HistoryEntry* nxtHistEntry);
		BeliefNode* addChild(long actIdx, ObsVals &obs);
		HistoryEntry* sampleAParticle();
		void updateVal(long actIdx, double newVal);
		void updateVal(long actIdx, double prevVal, double newVal, bool cutPart);
		//void delPartNUpdateVal(HistoryEntry *histEntryToBeDeleted, double prevQVal, double newVal);
		double distL1Independent(BeliefNode *b);
		
		BeliefNode* getChild(long actIdx, ObsVals &obs);
		void getChildren(queue<BeliefNode*> &res);
		void write(ostream &os);
		void writeNGetChildren(ostream &os, queue<BeliefNode*> &res);
		void writeStParticles(ostream &os);
		
		long getNxtActToTry();
		void calcBestVal();
//		void setEMDSig();

		inline long getId() { return id; }
		inline long getNParticles() { return nParticles; }
		inline long getNActChildren() { return nActChildren; }
		
	private:
		static long currId;
		static double exploreParam;
		static clock_t startTime;
		
		long id, nParticles, nActChildren, nxtActToTry;
		double bestAvgQVal;
		long bestAct;
		
		double tLastAddedParticle, tNNComp, tEmdSig;
		BeliefNode *nnBel;
		//CvMat *emdSig;

		
		vector<HistoryEntry*> particles;
		map<long, Action*> actChildren, invalidActChildren;
		
};
#endif
