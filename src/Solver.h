#ifndef SOLVER_H
#define SOLVER_H

#include <iostream>
#include <vector>
#include "Model.h"
#include "BeliefTree.h"
#include "Histories.h"
#include "State.h"
#include "StatePool.h"

using namespace std;

class Solver {
	public:
		Solver(Model *model_, BeliefTree *policy_, Histories *histories_);
		Solver(Model *model_, const char *polFile, BeliefTree *policy_, Histories *histories_);
		~Solver();
		
		void genPol(long maxTrials, double depthTh);
		double runSim(long nSteps, vector<long> &tChanges, vector<StateVals> &trajSt,
				vector<long> &trajActId, vector<ObsVals> &trajObs, vector<double> &trajRew, long *actualNSteps, double *totChTime, double *totImpTime);
		void write(ostream &os);
		
	private:
		Model *model;
		BeliefTree *policy;
		Histories *allHistories;
		StatePool *allStates;
		
		enum ModeRollout { ROLLOUT_RANDHEURISTIC=0, ROLLOUT_POL=1 };
		ModeRollout rolloutUsed;
		double exploreCoef;
		double cRollout[2], wRollout[2], pRollout[2];
		long nUsedRollout[2];
		
		//enum ModeModif { MODIF_REACHPOL, MODIF_UNREACHPOL, MODIF_ADDSEQ };
		
		
		void singleSearch(double discount, double depthTh);
		void singleSearch(BeliefNode *startNode, double discount, double depthTh);
		long getRolloutAct(BeliefNode *belNode, StateVals &s, double startDisc, double disc, StateVals &nxtSVals, ObsVals &obs, double *immediateRew, double *qVal);
		double rolloutPolHelper(BeliefNode *currNode, StateVals &s, double disc);
		//BeliefNode* getNNBelNodeEMD(BeliefNode *b);
		BeliefNode* getNNBelNode(BeliefNode *b);
		void updWeightRolloutAct(double valImprovement);
		void backup(HistorySeq *history);

		bool simAStep(StateVals& currStVals, StateVals &nxtStVals, BeliefNode **startNode, BeliefNode **nxtNode,
				double *rew, vector<StateVals> &trajSt, vector<long> &trajActId, vector<ObsVals> &trajObs); 
		/*
		void identifyAffectedPol(vector<StateVals> &affectedRage, BeliefNode *currNode, set<long> &affectedHistSeq);
		void identifyAffectedPol(vector<StateVals> &affectedRange, BeliefNode *currNode, 
				set<long> &reachAffectedHistSeq, set<long> &notReachAffectedHistSeq);
		*/
		void identifyAffectedPol(vector<StateVals> &affectedRage, vector<ChType> &chTypes, 
				BeliefNode *currNode, set<HistorySeq*> &affectedHistSeq);
				
		void resetAffected(set<HistorySeq*> affectedHistSeq);
		void updatePol(set<HistorySeq*> &affectedHistSeq);	
		void updateVal(HistorySeq *histSeq);
		void improveSol(BeliefNode* startNode, long maxTrials, double depthTh);
		BeliefNode* addChild(BeliefNode *currNode, long actId, ObsVals &obs, long timeStep);
		
		void removePathFrBelNode(HistorySeq *history);
		void modifHistSeqFr(HistorySeq *history, vector<StateVals> &modifStSeq, vector<long> &modifActSeq, vector<ObsVals> &modifObsSeq, vector<double> &modifRewSeq);
		void modifHistSeqFrTo(HistorySeq *history, vector<StateVals> &modifStSeq, vector<long> &modifActSeq, vector<ObsVals> &modifObsSeq, vector<double> &modifRewSeq);
	
};
#endif
