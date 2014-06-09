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
    friend class BasicSearchStrategy;
    friend class Serializer;
    friend class TextSerializer;

    Solver(std::unique_ptr<Model> model);
    ~Solver();

    _NO_COPY_OR_MOVE(Solver)
    ;

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

    /** Returns the estimation strategy. */
    EstimationStrategy *getEstimationStrategy() const;

    /** Returns the serializer for this solver. */
    Serializer *getSerializer() const;

    /* ------------------ Initialization methods ------------------- */
    /** Full initialization - resets all data structures. */
    void initializeEmpty();

    /* ------------------- Policy mutators ------------------- */
    /** Improves the policy by generating the given number of histories from
     * root node (-1 => default)
     * Histories are terminated upon reaching the maximum depth in the tree
     * (-1 => default)
     */
    void improvePolicy(long numberOfHistories = -1, long maximumDepth = -1);
    /** Improves the policy by generating the given number of histories from
     * the given belief node.
     */
    void improvePolicy(BeliefNode *startNode, long numberOfHistories = -1, long maximumDepth = -1);
    /** Applies any model changes that have been marked within the state pool */
    void applyChanges();
    /** Replenishes the particle count in the child node, ensuring that it
     * has at least the given number of particles
     * (-1 => default == model.getMinParticleCount())
     */
    BeliefNode *replenishChild(BeliefNode *currNode, Action const &action, Observation const &obs,
            long minParticleCount = -1);

    /* ------------------ Display methods  ------------------- */
    /** Shows a belief node in a nice, readable way. */
    void printBelief(BeliefNode *belief, std::ostream &os);

    /** Prints a compact representation of the entire tree. */
    void printTree(std::ostream &os);

    /* ------------------ Tree backup methods ------------------- */
    /** Returns true iff the tree has been properly backed up. */
    bool isBackedUp() const;
    /** Performs a backup on the entire tree. */
    void doBackup();

private:
    /* ------------------ Initialization methods ------------------- */
    /** Partial pre-initialization - helper for full initialization,
     *    and for loading from a file.
     */
    void initialize();

    /* ------------------ Episode sampling methods ------------------- */
    /** Runs multiple searches from the given start node and start states. */
    void multipleSearches(BeliefNode *node, std::vector<StateInfo *> states,
            long maximumDepth = -1);
    /** Searches from the given start node with the given start state. */
    void singleSearch(BeliefNode *startNode, StateInfo *startStateInfo, long maximumDepth = -1);
    /** Continues a pre-existing history sequence from its endpoint. */
    void continueSearch(HistorySequence *sequence, long maximumDepth = -1);

    /* ------------------ Tree backup methods ------------------- */
    /** Performs a negative backup for this entire sequence. */
    void negateSequence(HistorySequence *sequence);

    /** Updates the values for taking the given action and receiving the given
     * observation from the given belief node.
     *
     * deltaTotalQ - change in total reward due to immediate rewards
     * deltaNVisits - number of new visits (usually +1, 0, or -1)
     */
    void updateImmediate(BeliefNode *node, Action const &action, Observation const &observation,
            double deltaTotalQ, long deltaNVisits);

    /** Updates the estimated q-value for the previous action based on the
     * given belief.
     *
     * deltaTotalQ - change in heuristic value at this belief node.
     * deltaNContinuations - change in number of visits to this node that
     * continue onwards (and hence can be estimated using the q-value
     * this node).
     *
     */
    void updateEstimate(BeliefNode *node, double deltaTotalQ, long deltaNContinuations);

    /** Adds a new node that requires backing up. */
    void addNodeToBackup(BeliefNode *node);

    /* ------------------ Private data fields ------------------- */
    /** The random number generator used. */
    RandomGenerator *randGen_;

    /** The POMDP model */
    std::unique_ptr<Model> model_;

    /** The serializer to be used with this solver. */
    std::unique_ptr<Serializer> serializer_;

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
    /** The strategy to use when searching the tree. */
    std::unique_ptr<SearchStrategy> searchStrategy_;

    /** The strategy for estimating the value of a belief node based on actions from it. */
    std::unique_ptr<EstimationStrategy> estimationStrategy_;

    /** The nodes to be updated, sorted by depth (deepest first) */
    std::map<int, std::set<BeliefNode *>, std::greater<int>> nodesToBackup_;
};
} /* namespace solver */

#endif /* SOLVER_SOLVER_HPP_ */
