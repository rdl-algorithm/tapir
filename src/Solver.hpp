#ifndef SOLVER_HPP
#define SOLVER_HPP

#include <ostream>
#include <vector>
#include <set>

#include "ChangeType.hpp"
#include "Observation.hpp"
#include "State.hpp"
class BeliefNode;
class BeliefTree;
class Histories;
class HistoryEntry;
class HistorySequence;
class Model;
class StatePool;

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
            std::vector<State> &trajSt, std::vector<long> &trajActId,
            std::vector<Observation> &trajObs, std::vector<double> &trajRew,
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
    long getRolloutAct(BeliefNode *belNode, State &s, double startDisc,
            double disc, State &nxtSVals, Observation &obs,
            double *immediateRew, double *qVal);
    double rolloutPolHelper(BeliefNode *currNode, State &s, double disc);
    //BeliefNode* getNNBelNodeEMD(BeliefNode *b);
    BeliefNode* getNNBelNode(BeliefNode *b);
    void updWeightRolloutAct(double valImprovement);
    void backup(HistorySequence *history);

    bool simAStep(State& currStVals, State &nxtStVals, BeliefNode **startNode,
            BeliefNode **nxtNode, double *rew, std::vector<State> &trajSt,
            std::vector<long> &trajActId, std::vector<Observation> &trajObs);
    /*
     void identifyAffectedPol(std::vector<State> &affectedRage, BeliefNode *currNode, std::set<long> &affectedHistSeq);
     void identifyAffectedPol(std::vector<State> &affectedRange, BeliefNode *currNode,
     std::set<long> &reachAffectedHistSeq, std::set<long> &notReachAffectedHistSeq);
     */
    void identifyAffectedPol(std::vector<State> &affectedRage,
            std::vector<ChangeType> &chTypes,
            std::set<HistorySequence*> &affectedHistSeq);

    void resetAffected(std::set<HistorySequence*> affectedHistSeq);
    void updatePol(std::set<HistorySequence*> &affectedHistSeq);
    void updateVal(HistorySequence *histSeq);
    void improveSol(BeliefNode* startNode, long maxTrials, double depthTh);
    BeliefNode* addChild(BeliefNode *currNode, long actId, Observation &obs,
            long timeStep);

    void removePathFrBelNode(HistorySequence *history);
    void modifHistSeqFr(HistorySequence *history,
            std::vector<State> &modifStSeq, std::vector<long> &modifActSeq,
            std::vector<Observation> &modifObsSeq,
            std::vector<double> &modifRewSeq);
    void modifHistSeqFrTo(HistorySequence *history,
            std::vector<State> &modifStSeq, std::vector<long> &modifActSeq,
            std::vector<Observation> &modifObsSeq,
            std::vector<double> &modifRewSeq);

};

#endif /* SOLVER_HPP */
