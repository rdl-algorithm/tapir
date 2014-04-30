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
class BackpropagationStrategy;
class BeliefNode;
class BeliefTree;
class Histories;
class HistoryEntry;
class HistorySequence;
class ObservationPool;
class SearchStrategy;
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

    /* ------------------ Initialization methods ------------------- */
    /** Fully initializes the solver, creating an empty state. */
    void initializeEmpty();
    /** Initializes the solver; not necessarily in an empty state (e.g. after
     * loading from a file).
     */
    void initialize();
    /** Sets the serializer to be used by this solver. */
    void setSerializer(std::unique_ptr<Serializer> serializer);
    /** Saves the state of the solver to the given output stream. */
    void saveStateTo(std::ostream &os);
    /** Loads the state of the solver from the given input stream. */
    void loadStateFrom(std::istream &is);

    /* ------------------ Solution methods ------------------- */
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

    /* Stuff temporarily made public - Josh */
        /** Improves the solution with the root at the given node. */
    void improveSolution(BeliefNode *startNode, long historiesPerStep,
            long maximumDepth);
        /** Handles particle depletion during the simulation. */
    BeliefNode *addChild(BeliefNode *currNode, Action const &action,
            Observation const &obs,
            long timeStep);
    Model::StepResult simAStep(BeliefNode *currentBelief, State const &currentState);
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
    /** Calculates the discounted rewards from each entry to the end of
     * the sequence.
     */
    void calculateRewards(HistorySequence *sequence);
    /** Performs or negates a backup on the given sequence. */
    void backup(HistorySequence *sequence, bool backingUp);

    /* ------------------ Simulation methods ------------------- */
    /** Simulates a single step. */
    //Model::StepResult simAStep(BeliefNode *currentBelief, State const &currentState);
    /** Improves the solution with the root at the given node. */
    //void improveSolution(BeliefNode *startNode, long historiesPerStep,
    //        long maximumDepth);
    /** Handles particle depletion during the simulation. */
    //BeliefNode *addChild(BeliefNode *currNode, Action const &action,
    //        Observation const &obs,
    //        long timeStep);


    /* -------------- Methods for handling model changes --------------- */
    /** Does all the work required for changing the model. */
    void handleChanges(long timeStep,
            State const &currentState,
            std::vector<std::unique_ptr<State>> &stateHistory);
    /** Applies the model changes that have been marked within the state pool */
    void applyChanges();

    /* ------------------ Display methods  ------------------- */
    /** Shows a belief node in a nice, readable way. */
    //void printBelief(BeliefNode *belief, std::ostream &os);

    /* ------------------ Private data fields ------------------- */
    /** The random number generator used. */
    RandomGenerator *randGen_;
    /** The serializer to be used with this solver. */
    std::unique_ptr<Serializer> serializer_;

    /** The POMDP model */
    std::unique_ptr<Model> model_;

    /** The pool of states. */
    std::unique_ptr<StatePool> statePool_;
    /** The full collection of simulated histories. */
    std::unique_ptr<Histories> histories_;
    /** The tree that stores the policy */
    std::unique_ptr<BeliefTree> policy_;

    /** The pool of actions (used to generate action mappings) */
    std::unique_ptr<ActionPool> actionPool_;
    /** The pool of observations (used to generate observation mappings) */
    std::unique_ptr<ObservationPool> observationPool_;

    /** The history corrector. */
    std::unique_ptr<HistoryCorrector> historyCorrector_;

    /** The strategy to use when selecting nodes within the tree. */
    std::unique_ptr<SearchStrategy> selectionStrategy_;
    /** The strategy to use when rolling out. */
    std::unique_ptr<SearchStrategy> rolloutStrategy_;
    /** The strategy to use for backpropagation. */
    std::unique_ptr<BackpropagationStrategy> backpropagationStrategy_;
};
} /* namespace solver */

#endif /* SOLVER_SOLVER_HPP_ */
