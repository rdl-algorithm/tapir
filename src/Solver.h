#ifndef SOLVER_H
#define SOLVER_H

#include <ostream>
#include <vector>
#include <set>

#include "Model.h"
#include "BeliefTree.h"
#include "Histories.h"
#include "StateWrapper.h"
#include "StatePool.h"

class Solver {
public:
    Solver(Model *model);
    Solver(Model *model, const char *polFile);
    ~Solver();
    Solver(const Solver&) = delete;
    Solver(Solver&) = delete;
    Solver &operator=(const Solver&) = delete;
    Solver &operator=(Solver&) = delete;

    enum RolloutMode {
        ROLLOUT_RANDHEURISTIC = 0, ROLLOUT_POL = 1
    };

    void genPol(long maxTrials, double depthTh);
    double runSim(long nSteps, std::vector<long> &tChanges,
            std::vector<StateVals> &trajSt, std::vector<long> &trajActId,
            std::vector<ObsVals> &trajObs, std::vector<double> &trajRew,
            long *actualNSteps, double *totChTime, double *totImpTime);
    void write(std::ostream &os);

private:
    Model *model;
    BeliefTree *policy;
    Histories *allHistories;
    StatePool *allStates;

    RolloutMode rolloutUsed;
    double exploreCoef;
    double cRollout[2], wRollout[2], pRollout[2];
    long nUsedRollout[2];

    void singleSearch(double discount, double depthTh);
    void singleSearch(BeliefNode *startNode, double discount, double depthTh,
            HistoryEntry* startParticle);
    long getRolloutAct(BeliefNode *belNode, StateVals &s, double startDisc,
            double disc, StateVals &nxtSVals, ObsVals &obs,
            double *immediateRew, double *qVal);
    double rolloutPolHelper(BeliefNode *currNode, StateVals &s, double disc);
    //BeliefNode* getNNBelNodeEMD(BeliefNode *b);
    BeliefNode* getNNBelNode(BeliefNode *b);
    void updWeightRolloutAct(double valImprovement);
    void backup(HistorySequence *history);

    bool simAStep(StateVals& currStVals, StateVals &nxtStVals,
            BeliefNode **startNode, BeliefNode **nxtNode, double *rew,
            std::vector<StateVals> &trajSt, std::vector<long> &trajActId,
            std::vector<ObsVals> &trajObs);
    /*
     void identifyAffectedPol(std::vector<StateVals> &affectedRage, BeliefNode *currNode, std::set<long> &affectedHistSeq);
     void identifyAffectedPol(std::vector<StateVals> &affectedRange, BeliefNode *currNode,
     std::set<long> &reachAffectedHistSeq, std::set<long> &notReachAffectedHistSeq);
     */
    void identifyAffectedPol(std::vector<StateVals> &affectedRage,
            std::vector<Change> &chTypes,
            std::set<HistorySequence*> &affectedHistSeq);

    void resetAffected(std::set<HistorySequence*> affectedHistSeq);
    void updatePol(std::set<HistorySequence*> &affectedHistSeq);
    void updateVal(HistorySequence *histSeq);
    void improveSol(BeliefNode* startNode, long maxTrials, double depthTh);
    BeliefNode* addChild(BeliefNode *currNode, long actId, ObsVals &obs,
            long timeStep);

    void removePathFrBelNode(HistorySequence *history);
    void modifHistSeqFr(HistorySequence *history,
            std::vector<StateVals> &modifStSeq, std::vector<long> &modifActSeq,
            std::vector<ObsVals> &modifObsSeq,
            std::vector<double> &modifRewSeq);
    void modifHistSeqFrTo(HistorySequence *history,
            std::vector<StateVals> &modifStSeq, std::vector<long> &modifActSeq,
            std::vector<ObsVals> &modifObsSeq,
            std::vector<double> &modifRewSeq);

};
#endif
