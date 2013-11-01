#ifndef MODEL_H
#define MODEL_H

#include <ostream>
#include <vector>
#include <map>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

typedef std::vector<double> StateVals;
typedef std::vector<double> ObsVals;
enum ChType {
    UNDEFINED=0,
    ADDSTATE=1,
    REWARD=2,
    TRANSITION=3,
    DELSTATE=4,
    ADDOBSERVATION=5,
    ADDOBSTACLE=6
};

class Model {
	public:
		Model(po::variables_map vm) {
		    nParticles = vm["SBT.nParticles"].as<long>();
		    maxTrials = vm["SBT.maxTrials"].as<long>();
		    maxDistTry = vm["SBT.maxDistTry"].as<long>();

		    exploreCoef = vm["SBT.exploreCoef"].as<double>();
		    depthTh = vm["SBT.depthTh"].as<double>();
		    distTh = vm["SBT.distTh"].as<double>();

		    discount = vm["problem.discount"].as<double>();
        }

        // Note: Subclasses must initially calculate the following parameters:
        // nActions, nObservations, nStVars,
        // minVal, maxVal

        // Problem parameters
		inline double getDiscount() { return discount; }
		inline long getNActions() { return nActions; }
		inline long getNObservations() { return nObservations; }
		inline long getNStVars() { return nStVars; }

        // SBT parameters
		inline long getNParticles() { return nParticles; }
		inline long getMaxTrials() { return maxTrials; }
		inline long getMaxDistTry() { return maxDistTry; }
		inline double getDepthTh() { return depthTh; }
		inline double getExploreCoef() { return exploreCoef; }
		inline double getMinVal() { return minVal; }
		inline double getMaxVal() { return maxVal; }
		inline double getDistTh() { return distTh; }


		/* --------------- Start virtual functions ----------------- */

		/** Samples an initial state from the belief vector. */
		virtual void sampleAnInitState(StateVals& tmpStVals)=0;
		virtual bool isTerm(StateVals &sVals)=0;
		/** Approximates the q-value of a state */
		virtual void solveHeuristic(StateVals &sVals, double *qVal)=0;
		virtual double getDefaultVal()=0;

		virtual bool getNextState(StateVals &sVals, long actId,
		        StateVals &nxtSVals, ObsVals &obs)=0;
		virtual bool getNextState(StateVals &sVals, long actId,
		        double *immediateRew, StateVals &nxtSVals, ObsVals &obs)=0;
		virtual double getNextStateNRew(StateVals &sVals, long actId,
		        ObsVals &obs, bool &isTerm)=0;
		virtual double getReward(StateVals &sVals)=0;
		virtual double getReward(StateVals &sVals, long actId)=0;


		virtual void getStatesSeeObs(long actId, ObsVals &obs,
		        std::vector<StateVals> &partSt,
		        std::map<int, StateVals> &partNxtSt)=0;
		virtual void getStatesSeeObs(ObsVals &obs,
		        std::vector<StateVals> &posNxtSt)=0;

		virtual void setChanges(const char* chName,
		        std::vector<long> &chTime)=0;
		virtual void update(long tCh, std::vector<StateVals> &affectedRange,
		        std::vector<ChType> &typeOfChanges)=0;
		virtual bool modifStSeq(std::vector<StateVals> &seqStVals,
		        long startAffectedIdx, long endAffectedIdx,
				std::vector<StateVals> &modifStSeq,
				std::vector<long> &modifActSeq,
				std::vector<ObsVals> &modifObsSeq,
				std::vector<double> &modifRewSeq)=0;

		virtual void drawEnv(std::ostream &os)=0;

    protected:
        // Problem parameters.
        /** The discount factor for the POMDP. */
        double discount;
        /** The number of possible actions. */
        long nActions;
        /** The number of possible observations. */
        long nObservations;
        /** The number of state variables. */
        long nStVars;

        // SBT parameters
        /** The number of particles per belief. */
        long nParticles;
        /** ?? */
        long maxTrials;
        /** ?? */
        long maxDistTry;
        /** ?? */
        double depthTh;
        /** ?? */
        double exploreCoef;
        /** ?? */
        double minVal;
        /** ?? */
        double maxVal;
        /** ?? */
        double distTh;
};

#endif
