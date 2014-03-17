#ifndef SOLVER_SOLVER_HPP_
#define SOLVER_SOLVER_HPP_

#include <memory>                       // for unique_ptr
#include <set>                          // for set
#include <unordered_set>
#include <utility>                      // for pair
#include <vector>                       // for vector

#include "global.hpp"                     // for RandomGenerator

#include "abstract-problem/Action.hpp"                   // for Action
#include "abstract-problem/Model.hpp"                    // for Model, Model::StepResult
#include "abstract-problem/Observation.hpp"              // for Observation
#include "abstract-problem/State.hpp"

#include "changes/ChangeFlags.hpp"               // for ChangeFlags

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

    Solver(RandomGenerator *randGen, std::unique_ptr<Model> model);
    ~Solver();
    _NO_COPY_OR_MOVE(Solver);

    /** Resets and initializes the solver. */
    void initialize();
    /** Sets the serializer to be used by this solver. */
    void setSerializer(std::unique_ptr<Serializer> serializer);
    /** Saves the state of the solver to the given output stream. */
    void saveStateTo(std::ostream &os);
    /** Loads the state of the solver from the given input stream. */
    void loadStateFrom(std::istream &is);

    /** Generates a starting policy for the solver, by generating the given
     * number (historiesPerStep) of episodes, and terminating episodes when the
     * depth reaches maximumDepth.
     */
    void genPol(long historiesPerStep, long maximumDepth);
    /** Runs a single simulation up to a maximum of nSteps steps, and generating
     * historiesPerStep histories every step.
     * The return value is the resulting total discounted reward.
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
    double runSim(long nSteps, long historiesPerStep, std::vector<long> &changeTimes,
            std::vector<std::unique_ptr<State>> &trajSt,
            std::vector<std::unique_ptr<Action>> &trajAction,
            std::vector<std::unique_ptr<Observation>> &trajObs,
            std::vector<double> &trajRew, long *actualNSteps, double *totChTime,
            double *totImpTime);

    /** Attempts to find another belief node near enough to this one to be
     * suitable for the policy-based rollout heuristic.
     */
    BeliefNode *getNNBelNode(BeliefNode *belief,
                    double maxNnDistance, long maxNnComparisons);

    /* ------------------ Simple getters. ------------------- */
    /** Returns the policy. */
    BeliefTree *getPolicy();
    /** Returns the state pool. */
    StatePool *getStatePool();
    /** Returns the model. */
    Model *getModel();
    /** Returns the action pool. */
    ActionPool *getActionPool();
    /** Returns the observation pool. */
    ObservationPool *getObservationPool();

    void printBelief(BeliefNode *belief, std::ostream &os);
  private:
    /* ------------------ Episode sampling methods ------------------- */
    /** Searches from the root node for initial policy generation. */
    void singleSearch(long maximumDepth);
    /** Searches from the given start node with the given start state. */
    void singleSearch(BeliefNode *startNode, StateInfo *startStateInfo,
            long startDepth, long maximumDepth);
    /** Continues a pre-existing history sequence from its endpoint. */
    void continueSearch(HistorySequence *sequence, long maximumDepth);

    /* ------------------ Tree backup methods ------------------- */
    /** Checks the validity of a sequence w.r.t backup. */
    void checkSequence(HistorySequence *sequence, bool doBackup);
    /** Performs or negates a backup on the given sequence. */
    void backup(HistorySequence *sequence, bool doBackup);

    /* ------------------ Simulation methods ------------------- */
    /** Simulates a single step. */
    Model::StepResult simAStep(BeliefNode *currentBelief, State const &currentState);
    /** Handles particle depletion during the simulation. */
    BeliefNode *addChild(BeliefNode *currNode, Action const &action,
            Observation const &obs,
            long timeStep);
    /** Improves the solution, with the root at the given node. */
    void improveSol(BeliefNode *startNode, long historiesPerStep,
            long maximumDepth);

    /* ------------------ Methods for handling model changes ------------------- */
    /** Applies the model changes that have been marked within the state pool */
    void applyChanges();

    /* ------------------ Private data fields ------------------- */
    /** The random number generator used. */
    RandomGenerator *randGen_;
    /** The serializer to be used with this solver. */
    std::unique_ptr<Serializer> serializer_;

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

    /** The strategy to use when searching within the tree. */
    std::unique_ptr<SearchStrategy> searchStrategy_;
    /** The strategy to use when rolling out. */
    std::unique_ptr<SearchStrategy> rolloutStrategy_;
};
} /* namespace solver */

#endif /* SOLVER_SOLVER_HPP_ */
