#ifndef SOLVER_HPP
#define SOLVER_HPP

#include <memory>                       // for unique_ptr
#include <set>                          // for set
#include <vector>                       // for vector

#include "ChangeType.hpp"               // for ChangeType
#include "Model.hpp"                    // for Model and Model::StepResult
#include "Observation.hpp"              // for Observation
class BeliefNode;
class BeliefTree;
class Histories;
class HistoryEntry;
class HistorySequence;
class State;
class StateInfo;
class StatePool;

class Solver {
public:
    friend class Serializer;
    friend class TextSerializer;

    Solver(Model *model);
    ~Solver();
    Solver(const Solver&) = delete;
    Solver(Solver&&) = delete;
    Solver &operator=(const Solver&) = delete;
    Solver &operator=(Solver&&) = delete;

    enum RolloutMode {
        ROLLOUT_RANDHEURISTIC = 0, ROLLOUT_POL = 1
    };

    /** Generates a starting policy for the solver, by generating the given
     * number (maxTrials) of episodes, and terminating episodes when the
     * current discount reaches the maximum (depthTh).
     */
    void genPol(long maxTrials, double depthTh);
    /** Runs a single simulation up to a maximum of nSteps steps, returning
     * the total discounted reward.
     *
     * Also sets trajSt, trajActId, trajObs, and trajRew to be the sequences
     * of states, actions, observations, and immediate rewards;
     *
     * actualNSteps will be the actual number of steps, which will be
     * different to nSteps if a terminal state is reached early.
     * totChTime will be the total amount of time spent on changing the model.
     * totImpTime will be the total amount of time spent on generating new
     * episodes to improve the policy.
     */
    double runSim(long nSteps, std::vector<long> &tChanges,
            std::vector<std::unique_ptr<State> > &trajSt,
            std::vector<long> &trajActId, std::vector<Observation> &trajObs,
            std::vector<double> &trajRew, long *actualNSteps, double *totChTime,
            double *totImpTime);

private:
    Model *model;
    BeliefTree *policy;
    Histories *allHistories;
    StatePool *allStates;

    RolloutMode rolloutUsed;
    double exploreCoef;
    double cRollout[2], wRollout[2], pRollout[2];
    long nUsedRollout[2];

    /* ------------------ Episode sampling methods ------------------- */
    /** Searches from the root node for initial policy generation. */
    void singleSearch(double discountFactor, double depthTh);
    /** Searches from the given start node with the given start state. */
    void singleSearch(BeliefNode *startNode, StateInfo *startStateInfo,
            long startDepth, double discountFactor, double depthTh);
    /** Performs a backup on the given history sequence. */
    void backup(HistorySequence *history);

    /** Uses a rollout method to select an action and get results. */
    std::pair<Model::StepResult, double> getRolloutAction(BeliefNode *belNode,
            State &state, double startDiscount, double discountFactor);
    /** Attempts to find another belief node near enough to this one to be
     * suitable for the policy-based rollout heuristic.
     */
    BeliefNode *getNNBelNode(BeliefNode *b);
    /** Helper method for policy-based rollout. */
    double rolloutPolHelper(BeliefNode *currNode, State &s, double disc);
    /** Updates the overall weighting of the different heuristic strategies
     * based on their performance.
     */
    void updWeightRolloutAct(double valImprovement);

    /* ------------------ Simulation methods ------------------- */
    /** Simulates a single step. */
    Model::StepResult simAStep(BeliefNode *currentBelief, State &currentState);
    /** Handles particle depletion during the simulation. */
    BeliefNode* addChild(BeliefNode *currNode, long actId, Observation &obs,
            long timeStep);
    /** Improves the solution, with the root at the given node. */
    void improveSol(BeliefNode *startNode, long maxTrials, double depthTh);

    /* ------------------ Methods for handling model changes ------------------- */
    /** Identifies which parts of which history sequences are affected by the
     * changes to the model.
     */
    void identifyAffectedPol(std::vector<std::unique_ptr<State> > &affectedRage,
                std::vector<ChangeType> &chTypes,
                std::set<HistorySequence*> &affectedHistSeq);
    /** Updates the affected history sequences */
    void updatePol(std::set<HistorySequence*> &affectedHistSeq);
    /** Resets the affeted history sequences */
    void resetAffected(std::set<HistorySequence*> affectedHistSeq);

    /** Updates the q-values based on the changes to reward values
     * within a history sequence.
     */
    void updateVal(HistorySequence *histSeq);
    /** Undoes the effects of backing up a history sequence */
    void removePathFrBelNode(HistorySequence *history);

    void modifHistSeqFr(HistorySequence *history,
            std::vector<std::unique_ptr<State> > &modifStSeq,
            std::vector<long> &modifActSeq,
            std::vector<Observation> &modifObsSeq,
            std::vector<double> &modifRewSeq);
    void modifHistSeqFrTo(HistorySequence *history,
            std::vector<std::unique_ptr<State> > &modifStSeq,
            std::vector<long> &modifActSeq,
            std::vector<Observation> &modifObsSeq,
            std::vector<double> &modifRewSeq);
};

#endif /* SOLVER_HPP */
