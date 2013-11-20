#ifndef MODEL_H
#define MODEL_H

#include <ostream>
#include <vector>
#include <map>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

typedef std::vector<double> StateVals;
typedef std::vector<double> ObsVals;

enum class Change {
    UNDEFINED = 0,
    ADDSTATE = 1,
    REWARD = 2,
    TRANSITION = 3,
    DELSTATE = 4,
    ADDOBSERVATION = 5,
    ADDOBSTACLE = 6
};

class Model {
public:
    /** Default constructor;
     * retrieves some of the values that are universal for SBT / POMDPs
     */
    Model(po::variables_map vm) {
        nParticles = vm["SBT.nParticles"].as<long>();
        maxTrials = vm["SBT.maxTrials"].as<long>();
        maxDistTry = vm["SBT.maxDistTry"].as<long>();

        exploreCoef = vm["SBT.exploreCoef"].as<double>();
        depthTh = vm["SBT.depthTh"].as<double>();
        distTh = vm["SBT.distTh"].as<double>();

        discount = vm["problem.discount"].as<double>();
    }

    /** Destructor must be virtual */
    virtual ~Model() {
    }

    /* ------------------ Non-virtual getters ------------------ */
    /** Returns the POMDP discount factor. */
    inline double getDiscount() {
        return discount;
    }

    // SBT parameters
    /** Returns the maximum number of particles */
    inline unsigned long getNParticles() {
        return nParticles;
    }
    /** Returns the maximum number of trials to run. */
    inline long getMaxTrials() {
        return maxTrials;
    }
    /** Returns the maximum depth. */
    inline double getDepthTh() {
        return depthTh;
    }
    /** Returns the exploration coefficient used for rollouts.
     * ??
     */
    inline double getExploreCoef() {
        return exploreCoef;
    }

    /** Returns the maximum number of nodes to check when searching
     * for a nearest-neighbour belief node.
     */
    inline long getMaxDistTry() {
        return maxDistTry;
    }
    /** Returns the smallest allowable distance when searching for
     * a nearest-neighbour belief node.
     */
    inline double getDistTh() {
        return distTh;
    }

    /* --------------- Start virtual getters ----------------- */
    // Subclasses are necessarily required to define these methods.

    /** Returns the # of actions for this POMDP. */
    virtual unsigned long getNActions()=0;
    /** Returns the # of observations f {or this POMDP. */
    virtual unsigned long getNObservations()=0;
    /** Returns the number of state variables for this PODMP. */
    virtual unsigned long getNStVars()=0;
    /** Returns a lower bound on the q-value. */
    virtual double getMinVal()=0;
    /** Returns an upper bound on the q-value. */
    virtual double getMaxVal()=0;

    /* --------------- Start virtual functions ----------------- */
    /** Samples an initial state from the belief vector. */
    virtual void sampleAnInitState(StateVals &sVals)=0;
    /** Returns true iff the given state is terminal. */
    virtual bool isTerm(StateVals &sVals)=0;
    /** Approximates the q-value of a state */
    virtual void solveHeuristic(StateVals &sVals, double *qVal)=0;
    /** Returns the default reward */
    virtual double getDefaultVal()=0;

    /** Generates the next state, an observation, and the reward. */
    virtual bool getNextState(StateVals &sVals, unsigned long actId,
            double *immediateRew, StateVals &nxtSVals, ObsVals &obs)=0;
    /** Returns the reward for the given state. */
    virtual double getReward(StateVals &sVals)=0;
    /** Returns the reward for the given state and action. */
    virtual double getReward(StateVals &sVals, unsigned long actId)=0;

    /** Creates a new belief node based on the state particles of the
     * previous node, as well as on the action and observation.
     */
    virtual void getStatesSeeObs(unsigned long actId, ObsVals &obs,
            std::vector<StateVals> &partSt,
            std::vector<StateVals> &partNxtSt)=0;
    /** Creates a new belief node based only on the previous action and
     * observation, assuming a poorly-informed prior over previous states.
     *
     * This should only be used if the previous belief turns out to be
     * incompatible with the current observation.
     */
    virtual void getStatesSeeObs(unsigned long actId, ObsVals &obs,
            std::vector<StateVals> &partNxtSt)=0;

    /** Loads model changes from the given file. */
    virtual void setChanges(const char *chName, std::vector<long> &chTime)=0;
    /** Retrieves the states that are affected*/
    virtual void update(long tCh, std::vector<StateVals> &affectedRange,
            std::vector<Change> &typeOfChanges)=0;
    virtual bool modifStSeq(std::vector<StateVals> &seqStVals,
            unsigned long startAffectedIdx, unsigned long endAffectedIdx,
            std::vector<StateVals> &modifStSeq, std::vector<long> &modifActSeq,
            std::vector<ObsVals> &modifObsSeq,
            std::vector<double> &modifRewSeq)=0;

    virtual void dispAct(unsigned long actId, std::ostream &os)=0;
    virtual void dispState(StateVals &s, std::ostream &os)=0;
    virtual void dispObs(ObsVals &o, std::ostream &os)=0;
    virtual void drawEnv(std::ostream &os)=0;

protected:
    // Problem parameters.
    double discount;

    // SBT parameters
    unsigned long nParticles;
    long maxTrials;
    double depthTh;
    double exploreCoef;

    long maxDistTry;
    double distTh;
};

#endif
