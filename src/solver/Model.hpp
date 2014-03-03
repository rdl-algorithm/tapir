#ifndef SOLVER_MODEL_HPP_
#define SOLVER_MODEL_HPP_

#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "global.hpp"                     // for RandomGenerator

#include "indexing/StateIndex.hpp"
#include "geometry/Action.hpp"        // for Action
#include "geometry/State.hpp"                    // for State
#include "geometry/Observation.hpp"              // for Observation
#include "mappings/ActionPool.hpp"
#include "mappings/ObservationPool.hpp"

#include "ChangeFlags.hpp"               // for ChangeFlags
#include "TransitionParameters.hpp"
#include "HistoryCorrector.hpp"

namespace solver {
class BeliefNode;
class StatePool;

class Model {
  public:
    /** Constructor - stores the RandomGenerator for this model. */
    Model() = default;

    // Default destructor; copying and moving disallowed!
    virtual ~Model() = default;
    _NO_COPY_OR_MOVE(Model);

    virtual std::string getName() = 0;
    virtual RandomGenerator *getRandomGenerator() = 0;

    /* ---------- Virtual getters for important model parameters  ---------- */
    // POMDP parameters
    /** Returns the POMDP discount factor. */
    virtual double getDiscountFactor() = 0;

    /** Returns the number of state variables */
    virtual long getNStVars() = 0;
    /** Returns a lower bound on the q-value. */
    virtual double getMinVal() = 0;
    /** Returns an upper bound on the q-value. */
    virtual double getMaxVal() = 0;


    // SBT algorithm parameters
    /** Returns the default number of particles per belief - this number will
     * be generated if particle depletion occurs.
     */
    virtual long getNParticles() = 0;
    /** Returns the maximum number of trials (i.e. simulated episodes) to run
     * in a single time step.State
     */
    virtual long getMaxTrials() = 0;

    /** Returns the maximum depth allowed in the tree. */
    virtual long getMaximumDepth() = 0;

    /** Returns the exploration coefficient used for selecting a rollout
     * heuristic - this should be a value between 0 and 1, where a 1 results in
     * both heuristics being equally probable.
     * A reasonable default value is 0.5.
     */
    virtual double getHeuristicExploreCoefficient() = 0;
    /** Returns the exploration/exploitation ratio for UCB; higher
     * values mean more exploration.
     * A reasonable default value is 1.0.
     */
    virtual double getUcbExploreCoefficient() = 0;


    /** Returns the maximum number of nodes to check when searching
     * for a nearest-neighbor belief node.
     */
    virtual long getMaxNnComparisons() = 0;
    /** Returns the maximum allowable distance when searching for
     * a nearest-neighbor belief node.
     */
    virtual double getMaxNnDistance() = 0;


    /* --------------- The model interface proper ----------------- */
    /** Samples an initial state from the belief vector. */
    virtual std::unique_ptr<State> sampleAnInitState() = 0;
    /** Samples a state uniformly at random from all states. */
    virtual std::unique_ptr<State> sampleStateUniform() = 0;
    /** Returns true iff the given state is terminal. */
    virtual bool isTerminal(State const &state) = 0;
    /** Approximates the q-value of a state */
    virtual double getHeuristicValue(State const &state) = 0;
    /** Returns the default q-value */
    virtual double getDefaultVal() = 0;

    /* --------------- Black box dynamics ----------------- */
    /** Represents the results of a complete step in the model,
     * including the next state, observation, and reward.
     */
    struct StepResult {
        std::unique_ptr<Action> action = 0;
        std::unique_ptr<TransitionParameters> transitionParameters = nullptr;
        std::unique_ptr<Observation> observation = nullptr;
        double reward = 0;
        std::unique_ptr<State> nextState = nullptr;
        bool isTerminal = false;
    };
    /** Generates the next state, an observation, and the reward. */
    virtual StepResult generateStep(State const &state,
            Action const &action) = 0;

    /** Generates the parameters for a next-state transition, if any are being
     * used - the default implementation simply returns nullptr.
     */
    virtual std::unique_ptr<TransitionParameters> generateTransition(
            State const &state,
            Action const &action);

    /** Generates the next state, based on the state and action, and,
     * if used, the transition parameters.
     */
    virtual std::unique_ptr<State> generateNextState(
            State const &state,
            Action const &action,
            TransitionParameters const *transitionParameters // optional
            ) = 0;

    /** Generates an observation, given the action and resulting next state;
     * optionally, the previous state and the transition parameters can also be
     * used.
     */
    virtual std::unique_ptr<Observation> generateObservation(
            State const *state, // optional
            Action const &action,
            TransitionParameters const *transitionParameters, // optional
            State const &nextState) = 0;

    /** Returns the reward for the given state, action; optionally this also
     * includes transition parameters and the next state - if they aren't
     * being used it is OK to use nullptr for those inputs.
     */
    virtual double generateReward(
            State const &state,
            Action const &action,
            TransitionParameters const *transitionParameters, // optional
            State const *nextState) = 0; // optional

    /* ------------ Methods for handling particle depletion -------------- */
    /** Generates new state particles based on the state particles of the
     * previous node, as well as on the action and observation.
     */
    virtual std::vector<std::unique_ptr<State>> generateParticles(
            BeliefNode *previousBelief,
            Action const &action, Observation const &obs,
            std::vector<State const *> const &previousParticles);
    /** Generates new state particles based only on the previous action and
     * observation, assuming a poorly-informed prior over previous states.
     *
     * This should only be used if the previous belief turns out to be
     * incompatible with the current observation.
     */
    virtual std::vector<std::unique_ptr<State>> generateParticles(
            BeliefNode *previousBelief,
            Action const &action, Observation const &obs);

    /* -------------- Methods for handling model changes ---------------- */
    /** Loads future model changes from the given file. */
    virtual std::vector<long> loadChanges(char const *changeFilename) = 0;
    /** Updates the model to reflect the changes at the given time,
     * and marks the affected states within the state pool.
     */
    virtual void update(long time, StatePool *pool) = 0;

    /* --------------- Pretty printing methods ----------------- */
    /** Draws the environment map onto the given output stream. */
    virtual void drawEnv(std::ostream &/*os*/) {}
    /** Draws the current belief and/or the current state in the context of the
     * overall map onto the given output stream.
     */
    virtual void drawSimulationState(std::vector<State const *> /*particles*/,
            State const &/*state*/, std::ostream &/*os*/) {}

    /* --------------- Data structure customization ----------------- */
    // These are factory methods to allow the data structures used to
    // be managed in a customizable way.

    /** Creates a StateIndex, which manages searching for states that
     * have been used in a StatePool.
     */
    virtual std::unique_ptr<StateIndex> createStateIndex();
    /** Creates an ActionPool, which manages actions & creates
     * ActionMappings
     */
    virtual std::unique_ptr<ActionPool> createActionPool() = 0;
    /** Creates an ObservationPool, which manages observations & creates
     * ObservationMappings.
     */
    virtual std::unique_ptr<ObservationPool> createObservationPool() = 0;
    /** Creates a HistoryCorrector; defaults to general one, but can
     * be made problem-specific.
     */
    virtual std::unique_ptr<HistoryCorrector> createHistoryCorrector();
};
} /* namespace solver */

#endif /* SOLVER_MODEL_HPP_ */
