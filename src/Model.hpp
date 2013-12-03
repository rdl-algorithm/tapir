#ifndef MODEL_HPP
#define MODEL_HPP

#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "ChangeType.hpp"               // for ChangeType
#include "Observation.hpp"              // for Observation
class State;

class Model {
public:
    /** Destructor must be virtual */
    virtual ~Model() = default;
    Model(const Model&) = delete;
    Model(Model&&) = delete;
    Model &operator=(const Model&) = delete;
    Model &operator=(Model&&) = delete;

    /* ---------- Virtual getters for important model parameters  ---------- */
    // POMDP parameters
    /** Returns the POMDP discount factor. */
    virtual double getDiscount() = 0;
    /** Returns the # of actions for this POMDP. */
    virtual unsigned long getNActions() = 0;
    /** Returns the # of observations f {or this POMDP. */
    virtual unsigned long getNObservations() = 0;
    /** Returns the number of state variables for this PODMP. */
    virtual unsigned long getNStVars() = 0;
    /** Returns a lower bound on the q-value. */
    virtual double getMinVal() = 0;
    /** Returns an upper bound on the q-value. */
    virtual double getMaxVal() = 0;

    // SBT algorithm parameters
    /** Returns the maximum number of particles */
    virtual unsigned long getNParticles() = 0;
    /** Returns the maximum number of trials to run. */
    virtual long getMaxTrials() = 0;
    /** Returns the lowest cumulative discount before the  */
    virtual double getDepthTh() = 0;
    /** Returns the exploration coefficient used for rollouts.
     * ??
     */
    virtual double getExploreCoef() = 0;
    /** Returns the maximum number of nodes to check when searching
     * for a nearest-neighbour belief node.
     */
    virtual long getMaxDistTry() = 0;
    /** Returns the smallest allowable distance when searching for
     * a nearest-neighbour belief node.
     */
    virtual double getDistTh() = 0;

    /* --------------- Start virtual functions ----------------- */
    /** Samples an initial state from the belief vector. */
    virtual std::unique_ptr<State> sampleAnInitState() = 0;
    /** Returns true iff the given state is terminal. */
    virtual bool isTerm(State &state) = 0;
    /** Approximates the q-value of a state */
    virtual double solveHeuristic(State &state) = 0;
    /** Returns the default q-value */
    virtual double getDefaultVal() = 0;

    /** Represents the results of a step in the model, including the next state,
     * observation, and reward.
     */
    struct StepResult {
        long action;
        Observation observation;
        double immediateReward;
        std::unique_ptr<State> nextState;
        bool isTerminal;
    };
    /** Generates the next state, an observation, and the reward. */
    virtual StepResult generateStep(State &state, unsigned long action) = 0;
    /** Returns the reward for the given state. */
    virtual double getReward(State &state) = 0;
    /** Returns the reward for the given state and action. */
    virtual double getReward(State &state, unsigned long action) = 0;

    /** Generates new state particles based on the state particles of the
     * previous node, as well as on the action and observation.
     */
    virtual std::vector<std::unique_ptr<State>> generateParticles(unsigned long action,
            Observation &obs, std::vector<State*> &previousParticles) = 0;
    /** Generates new state particles based only on the previous action and
     * observation, assuming a poorly-informed prior over previous states.
     *
     * This should only be used if the previous belief turns out to be
     * incompatible with the current observation.
     */
    virtual std::vector<std::unique_ptr<State>> generateParticles(unsigned long action,
            Observation &obs) = 0;

    /** Loads model changes from the given file. */
    virtual std::vector<long> loadChanges(const char *chName) = 0;

    /** Retrieves the range of states that is affected by the change. */
    virtual void update(long tCh, std::vector<std::unique_ptr<State> > &affectedRange,
            std::vector<ChangeType> &typeOfChanges) = 0;

    /** Generates a modified version of the given sequence of states, between
     * the start and end indices.
     *
     * Should return true if modifications have actually been made, and false
     * otherwise.
     */
    virtual bool modifStSeq(std::vector<State*> &states,
            long startAffectedIdx, long endAffectedIdx,
            std::vector<std::unique_ptr<State> > &modifStSeq, std::vector<long> &modifActSeq,
            std::vector<Observation> &modifObsSeq,
            std::vector<double> &modifRewSeq) = 0;

    virtual void dispAct(unsigned long actId, std::ostream &os) = 0;
    virtual void dispObs(Observation &o, std::ostream &os) = 0;
    virtual void drawEnv(std::ostream &os) = 0;
    virtual void drawState(State &s, std::ostream &os) = 0;
};

#endif
