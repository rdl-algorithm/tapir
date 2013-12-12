#ifndef SOLVER_MODEL_HPP_
#define SOLVER_MODEL_HPP_

#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "defs.hpp"                     // for RandomGenerator

#include "Action.hpp"                   // for Action
#include "ChangeType.hpp"               // for ChangeType
#include "Observation.hpp"              // for Observation
#include "State.hpp"                    // for State

namespace solver {
class Model {
  public:
    /** Represents the results of a step in the model, including the next state,
     * observation, and reward.
     */
    struct StepResult {
        Action action = 0;
        Observation observation = Observation();
        double immediateReward = 0;
        std::unique_ptr<State> nextState = std::unique_ptr<State>();
        bool isTerminal = false;
    };

    /** Constructor - stores the RandomGenerator for this model. */
    Model(RandomGenerator *randGen) : randGen_(randGen) {
    }

    /** Destructor must be virtual */
    virtual ~Model() = default;
    /** Moving and copying is disallowed. */
    Model(Model const &) = delete;
    Model(Model &&) = delete;
    Model &operator=(Model const &) = delete;
    Model &operator=(Model &&) = delete;

    /* ---------- Virtual getters for important model parameters  ---------- */
    // POMDP parameters
    /** Returns the POMDP discount factor. */
    virtual double getDiscountFactor() = 0;

    /** Returns the # of actions for this POMDP. */
    virtual unsigned long getNActions() = 0;
    /** Returns the # of observations for this POMDP. */
    virtual unsigned long getNObservations() = 0;
    /** Returns the number of state variables for this PODMP. */
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


    /* --------------- Start virtual functions ----------------- */
    /** Samples an initial state from the belief vector. */
    virtual std::unique_ptr<State> sampleAnInitState() = 0;
    /** Returns true iff the given state is terminal. */
    virtual bool isTerm(State const &state) = 0;
    /** Approximates the q-value of a state */
    virtual double solveHeuristic(State const &state) = 0;
    /** Returns the default q-value */
    virtual double getDefaultVal() = 0;

    /** Generates the next state, an observation, and the reward. */
    virtual StepResult generateStep(State const &state,
            Action const &action) = 0;
    /** Returns the reward for the given state. */
    virtual double getReward(State const &state) = 0;
    /** Returns the reward for the given state and action. */
    virtual double getReward(State const &state, Action const &action) = 0;

    /** Generates new state particles based on the state particles of the
     * previous node, as well as on the action and observation.
     */
    virtual std::vector<std::unique_ptr<State>> generateParticles(
            Action const &action,
            Observation const &obs,
            std::vector<State *> const &previousParticles) = 0;
    /** Generates new state particles based only on the previous action and
     * observation, assuming a poorly-informed prior over previous states.
     *
     * This should only be used if the previous belief turns out to be
     * incompatible with the current observation.
     */
    virtual std::vector<std::unique_ptr<State>> generateParticles(
            Action const &,
            Observation const &obs) = 0;

    /** Loads future model changes from the given file. */
    virtual std::vector<long> loadChanges(char const *changeFilename) = 0;

    /** Updates the model to reflect the changes at the given time, and
     * returns the range of states that is affected by the change.
     */
    virtual void update(long time,
            std::vector<std::unique_ptr<State>> *affectedRange,
            std::vector<ChangeType> *typeOfChanges) = 0;

    /** Generates a modified version of the given sequence of states, between
     * the start and end indices.
     *
     * Should return true if modifications have actually been made, and false
     * otherwise.
     */
    virtual bool modifStSeq(std::vector<State const *> const &states,
            long startAffectedIdx, long endAffectedIdx,
            std::vector<std::unique_ptr<State>> *modifStSeq,
            std::vector<Action> *modifActSeq,
            std::vector<Observation> *modifObsSeq,
            std::vector<double> *modifRewSeq) = 0;

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
