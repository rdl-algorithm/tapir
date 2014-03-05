#ifndef SOLVER_SOLVER_HPP_
#define SOLVER_SOLVER_HPP_

#include <memory>                       // for unique_ptr
#include <set>                          // for set
#include <unordered_set>
#include <utility>                      // for pair
#include <vector>                       // for vector

#include "global.hpp"                     // for RandomGenerator

#include "geometry/Action.hpp"                   // for Action
#include "ChangeFlags.hpp"               // for ChangeFlags
#include "Model.hpp"                    // for Model, Model::StepResult
#include "geometry/Observation.hpp"              // for Observation
#include "geometry/State.hpp"

namespace nav2d{
class Nav2DSpcHistoryCorrector;
}

namespace solver {
class ActionPool;
class BeliefNode;
class BeliefTree;
class Histories;
class HistoryEntry;
class HistorySequence;
class ObservationPool;
class Serializer;
class StateInfo;
class StatePool;

class Solver {
  public:
    friend class Serializer;
    friend class TextSerializer;
    friend class EnumeratedActionTextSerializer;
    friend class DiscretizedActionTextSerializer;
    friend class ApproximateObservationTextSerializer;
    friend class DiscreteObservationTextSerializer;
    friend class EnumeratedObservationTextSerializer;

    friend class DefaultHistoryCorrector;
    friend class nav2d::Nav2DSpcHistoryCorrector;

    Solver(RandomGenerator *randGen, std::unique_ptr<Model> model);
    ~Solver();
    _NO_COPY_OR_MOVE(Solver);

    enum RolloutMode {
        ROLLOUT_RANDHEURISTIC = 0,
        ROLLOUT_POL = 1
    };

    /** Resets and initializes the solver. */
    void initialize();

    /** Sets the serializer to be used by this solver. */
    void setSerializer(Serializer *serializer);

    /** Generates a starting policy for the solver, by generating the given
     * number (maxTrials) of episodes, and terminating episodes when the
     * depth reaches maximumDepth.
     */
    void genPol(long maxTrials, long maximumDepth);
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
    double runSim(long nSteps, std::vector<long> &changeTimes,
            std::vector<std::unique_ptr<State>> &trajSt,
            std::vector<std::unique_ptr<Action>> &trajAction,
            std::vector<std::unique_ptr<Observation>> &trajObs,
            std::vector<double> &trajRew, long *actualNSteps, double *totChTime,
            double *totImpTime);

  private:
    /* ------------------ Episode sampling methods ------------------- */
    /** Searches from the root node for initial policy generation. */
    void singleSearch(double discountFactor, long maximumDepth);
    /** Searches from the given start node with the given start state. */
    void singleSearch(BeliefNode *startNode, StateInfo *startStateInfo,
            long startDepth, double discountFactor, long maximumDepth);
    /** Continues a pre-existing history sequence from its endpoint. */
    void continueSearch(HistorySequence *sequence,
            double discountFactor, long maximumDepth);
    /** Performs a backup on the given history sequence. */
    void backup(HistorySequence *sequence);

    /** Uses a rollout method to select an action and get results. */
    std::pair<Model::StepResult, double> getRolloutAction(BeliefNode *belNode,
            State const &state, double startDiscount, double discountFactor);
    /** Attempts to find another belief node near enough to this one to be
     * suitable for the policy-based rollout heuristic.
     */
    BeliefNode *getNNBelNode(BeliefNode *b);
    /** Helper method for policy-based rollout. */
    double rolloutPolHelper(BeliefNode *currNode, State const &state, double disc);
    /** Updates the overall weighting of the different heuristic heuristics
     * based on their performance.
     */
    void updateHeuristicProbabilities(double valImprovement);

    /* ------------------ Simulation methods ------------------- */
    /** Simulates a single step. */
    Model::StepResult simAStep(BeliefNode *currentBelief, State const &currentState);
    /** Handles particle depletion during the simulation. */
    BeliefNode *addChild(BeliefNode *currNode, Action const &action,
            Observation const &obs,
            long timeStep);
    /** Improves the solution, with the root at the given node. */
    void improveSol(BeliefNode *startNode, long maxTrials,
            long maximumDepth);

    /* ------------------ Methods for handling model changes ------------------- */
    void applyChanges();
    void fixLinks(HistorySequence *sequence);
    /** Negates a backup on the given history sequence. */
    void undoBackup(HistorySequence *sequence);

    /* ------------------ Private data fields ------------------- */
    /** The serializer to be used with this solver. */
    Serializer *serializer_;
    /** The random number generator used. */
    RandomGenerator *randGen_;
    /** The POMDP model */
    std::unique_ptr<Model> model_;

    /** The pool of actions. */
    std::unique_ptr<ActionPool> actionPool_;
    /** The pool of observations. */
    std::unique_ptr<ObservationPool> observationPool_;

    /** The pool of states. */
    std::unique_ptr<StatePool> allStates_;
    /** The full collection of simulated histories. */
    std::unique_ptr<Histories> allHistories_;
    /** The tree that stores the policy */
    std::unique_ptr<BeliefTree> policy_;

    /** The history corrector. */
    std::unique_ptr<HistoryCorrector> historyCorrector_;

    /** The rollout mode that was used in the last rollout. */
    RolloutMode lastRolloutMode_;
    /** The coefficient that determines how much to explore heuristics
     * vs. exploiting them.
     */
    double heuristicExploreCoefficient_;
    /** The time used by each rollout heuristic. */
    double timeUsedPerHeuristic_[2];
    /** The weight associated with each rollout heuristic. */
    double heuristicWeight_[2];
    /** The calculated probability for the usage of each rollout heuristic. */
    double heuristicProbability_[2];
    /** The number of times each rollout heuristic has been used. */
    long heuristicUseCount_[2];
};
} /* namespace solver */

#endif /* SOLVER_SOLVER_HPP_ */
