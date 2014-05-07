#ifndef SOLVER_SOLVER_HPP_
#define SOLVER_SOLVER_HPP_

#include <map>
#include <memory>        // for unique_ptr
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

    /* ------------------ Simple getters. ------------------- */
    /** Returns the policy. */
    BeliefTree *getPolicy() const;
    /** Returns the state pool. */
    StatePool *getStatePool() const;
    /** Returns the model. */
    Model *getModel() const;
    /** Returns the action pool. */
    ActionPool *getActionPool() const;
    /** Returns the observation pool. */
    ObservationPool *getObservationPool() const;

    /* ------------------ Initialization methods ------------------- */
    /** Full initialization - resets all data structures. */
    void initializeEmpty();
    /** Returns the serializer for this solver. */
    Serializer *getSerializer();
    /** Sets the serializer to be used by this solver. */
    void setSerializer(std::unique_ptr<Serializer> serializer);

    /* ------------------- Policy mutators ------------------- */
    /** Improves the policy by generating the given number of histories from
     * root node (-1 => default)
     * Histories are terminated upon reaching the maximum depth in the tree
     * (-1 => default)
     */
    void improvePolicy(long numberOfHistories=-1, long maximumDepth=-1);
    /** Improves the policy by generating the given number of histories from
     * the given belief node.
     */
    void improvePolicy(BeliefNode *startNode, long numberOfHistories=-1,
            long maximumDepth=-1);
    /** Applies any model changes that have been marked within the state pool */
    void applyChanges();
    /** Handles particle depletion during the simulation. */
    BeliefNode *addChild(BeliefNode *currNode, Action const &action,
            Observation const &obs);

    /* ------------------ Display methods  ------------------- */
    /** Shows a belief node in a nice, readable way. */
    void printBelief(BeliefNode *belief, std::ostream &os);

    /* ------------------ Simulation methods ------------------- */
    /** Runs a single simulation up to a maximum of nSteps steps;
     * the return value is the resulting total discounted reward.
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
    double runSimulation(long nSteps,
            std::map<long, std::vector<std::unique_ptr<ModelChange>>> changeSequence,
            std::vector<std::unique_ptr<State>> &trajSt,
            std::vector<std::unique_ptr<Action>> &trajAction,
            std::vector<std::unique_ptr<Observation>> &trajObs,
            std::vector<double> &trajRew,
            long *actualNSteps,
            double *totChTime,
            double *totImpTime);

private:
    /* ------------------ Initialization methods ------------------- */
    /** Partial pre-initialization - helper for full initialization,
     *    and for loading from a file.
     */
    void initialize();

    /* ------------------ Episode sampling methods ------------------- */
    /** Runs multiple searches from the given start node and start states. */
    void multipleSearches(BeliefNode *node, std::vector<StateInfo *> states,
            long maximumDepth=-1);
    /** Searches from the given start node with the given start state. */
    void singleSearch(BeliefNode *startNode, StateInfo *startStateInfo,
            long maximumDepth=-1);
    /** Continues a pre-existing history sequence from its endpoint. */
    void continueSearch(HistorySequence *sequence,
            long maximumDepth=-1);

    /* ------------------ Tree backup methods ------------------- */
    /** Calculates the discounted rewards from each entry to the end of
     * the sequence.
     */
    void calculateRewards(HistorySequence *sequence);
    /** Performs or negates a backup on the given sequence. */
    void backup(HistorySequence *sequence, bool backingUp);

    /* ------------------ Simulation methods ------------------- */
    /** Simulates a single step. */
    Model::StepResult simAStep(BeliefNode *currentBelief,
            State const &currentState);

    /* -------------- Methods for handling model changes --------------- */
    /** Does all the work required for changing the model. */
    void handleChanges(std::vector<std::unique_ptr<ModelChange>> changes,
            State const &currentState,
            std::vector<std::unique_ptr<State>> &stateHistory);

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
