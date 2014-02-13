#ifndef SOLVER_MODEL_HPP_
#define SOLVER_MODEL_HPP_

#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "defs.hpp"                     // for RandomGenerator

#include "Action.hpp"                   // for Action
#include "ChangeFlags.hpp"               // for ChangeFlags
#include "Observation.hpp"              // for Observation
#include "State.hpp"                    // for State

namespace solver {
class StatePool;

class Model {
  public:
    /** Constructor - stores the RandomGenerator for this model. */
    Model(RandomGenerator *randGen) : randGen_(randGen) {
    }

    // Default destructor; copying and moving disallowed!
    virtual ~Model() = default;
    Model(Model const &) = delete;
    Model(Model &&) = delete;
    Model &operator=(Model const &) = delete;
    Model &operator=(Model &&) = delete;

    virtual std::string getName() = 0;

    /* ---------- Virtual getters for important model parameters  ---------- */
    // POMDP parameters
    /** Returns the POMDP discount factor. */
    virtual double getDiscountFactor() = 0;

    /** Returns the # of discrete actions for this POMDP. */
    virtual unsigned long getNActions() = 0;
    /** Returns the number of state variables */
    virtual unsigned long getNStVars() = 0;
    /** Returns a lower bound on the q-value. */
    virtual double getMinVal() = 0;
    /** Returns an upper bound on the q-value. */
    virtual double getMaxVal() = 0;


    // SBT algorithm parameters
    /** Returns the default number of particles per belief - this number will
     * be generated if particle depletion occurs.
     */
    virtual unsigned long getNParticles() = 0;
    /** Returns the maximum number of trials (i.e. simulated episodes) to run
     * in a single time step.
     */
    virtual unsigned long getMaxTrials() = 0;
    /** Returns the lowest net discount allowed; tree searching will not go
     * deeper than this threshold.
     */
    virtual double getMinimumDiscount() = 0;


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
    /** Returns true iff the given state is terminal. */
    virtual bool isTerminal(State const &state) = 0;
    /** Approximates the q-value of a state */
    virtual double getHeuristicValue(State const &state) = 0;
    /** Returns the default q-value */
    virtual double getDefaultVal() = 0;

    /* --------------- Black box dynamics ----------------- */
    /** Generates a next state from the previous state and the action taken. */
    virtual std::unique_ptr<State> generateNextState(State const &state,
            Action const &action) = 0;
    /** Generates an observation, given the action and resulting next state. */
    virtual std::unique_ptr<Observation> generateObservation(Action const &action,
            State const &nextState) = 0;
    /** Returns the reward for the given state and action. */
    virtual double getReward(State const &state, Action const &action) = 0;

    /** Represents the results of a complete step in the model,
     * including the next state, observation, and reward.
     */
    struct StepResult {
        Action action = 0;
        std::unique_ptr<Observation> observation = nullptr;
        double immediateReward = 0;
        std::unique_ptr<State> nextState = nullptr;
        bool isTerminal = false;
    };
    /** Generates the next state, an observation, and the reward. */
     virtual StepResult generateStep(State const &state,
            Action const &action) = 0;

    /* ------------ Methods for handling particle depletion -------------- */
    /** Generates new state particles based on the state particles of the
     * previous node, as well as on the action and observation.
     */
    virtual std::vector<std::unique_ptr<State>> generateParticles(
            Action const &action, Observation const &obs,
            std::vector<State const *> const &previousParticles) = 0;
    /** Generates new state particles based only on the previous action and
     * observation, assuming a poorly-informed prior over previous states.
     *
     * This should only be used if the previous belief turns out to be
     * incompatible with the current observation.
     */
    virtual std::vector<std::unique_ptr<State>> generateParticles(
            Action const &, Observation const &obs) = 0;

    /* -------------- Methods for handling model changes ---------------- */
    /** Loads future model changes from the given file. */
    virtual std::vector<long> loadChanges(char const *changeFilename) = 0;
    /** Updates the model to reflect the changes at the given time,
     * and marks the affected states within the state pool.
     */
    virtual void update(long time, StatePool *pool) = 0;

    /* --------------- Pretty printing methods ----------------- */
    /** Displays the given action on the given output stream. */
    virtual void dispAct(Action const &action, std::ostream &os) = 0;
    /** Displays the given observation to the given output stream. */
    virtual void dispObs(Observation const &obs, std::ostream &os) = 0;
    /** Draws the environment map onto the given output stream. */
    virtual void drawEnv(std::ostream &os) = 0;
    /** Draws the current state in the context of the overall map onto
     * the given output stream.
     */
    virtual void drawState(State const &state, std::ostream &os) = 0;
  protected:
    /** The random number generator for this model. */
    RandomGenerator *randGen_;
};
} /* namespace solver */

#endif /* SOLVER_MODEL_HPP_ */
