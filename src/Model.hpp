#ifndef MODEL_HPP
#define MODEL_HPP

#include <iosfwd>
#include <vector>

#include "ChangeType.hpp"
#include "Observation.hpp"
#include "State.hpp"

class Model {
public:
    /** Destructor must be virtual */
    virtual ~Model() = default;

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
    virtual void sampleAnInitState(State &sVals) = 0;
    /** Returns true iff the given state is terminal. */
    virtual bool isTerm(State &sVals) = 0;
    /** Approximates the q-value of a state */
    virtual void solveHeuristic(State &sVals, double *qVal) = 0;
    /** Returns the default q-value */
    virtual double getDefaultVal() = 0;

    /** Generates the next state, an observation, and the reward. */
    virtual bool getNextState(State &sVals, unsigned long actId,
            double *immediateRew, State &nxtSVals, Observation &obs) = 0;
    /** Returns the reward for the given state. */
    virtual double getReward(State &sVals) = 0;
    /** Returns the reward for the given state and action. */
    virtual double getReward(State &sVals, unsigned long actId) = 0;

    /** Creates a new belief node based on the state particles of the
     * previous node, as well as on the action and observation.
     */
    virtual void getStatesSeeObs(unsigned long actId, Observation &obs,
            std::vector<State> &partSt, std::vector<State> &partNxtSt) = 0;
    /** Creates a new belief node based only on the previous action and
     * observation, assuming a poorly-informed prior over previous states.
     *
     * This should only be used if the previous belief turns out to be
     * incompatible with the current observation.
     */
    virtual void getStatesSeeObs(unsigned long actId, Observation &obs,
            std::vector<State> &partNxtSt) = 0;

    /** Loads model changes from the given file. */
    virtual void setChanges(const char *chName, std::vector<long> &chTime) = 0;
    /** Retrieves the states that are affected*/
    virtual void update(long tCh, std::vector<State> &affectedRange,
            std::vector<ChangeType> &typeOfChanges) = 0;
    virtual bool modifStSeq(std::vector<State> &seqStVals,
            long startAffectedIdx, long endAffectedIdx,
            std::vector<State> &modifStSeq, std::vector<long> &modifActSeq,
            std::vector<Observation> &modifObsSeq,
            std::vector<double> &modifRewSeq) = 0;

    virtual void dispAct(unsigned long actId, std::ostream &os) = 0;
    virtual void dispState(State &s, std::ostream &os) = 0;
    virtual void dispObs(Observation &o, std::ostream &os) = 0;
    virtual void drawEnv(std::ostream &os) = 0;
    virtual void drawState(State &s, std::ostream &os) = 0;
};

#endif
