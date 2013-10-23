#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <map>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace std;

typedef vector<double> StateVals;
typedef vector<double> ObsVals;
enum ChType { UNDEFINED=0, ADDSTATE=1, REWARD=2, TRANSITION=3, DELSTATE=4 , ADDOBSERVATION=5, ADDOBSTACLE=6 };

class Model {
	public:
		long nParticles, nActions;
		vector<StateVals> initBel;
		long nInitBel, maxTrials, nStVars, maxDistTry;
		double discount, depthTh, exploreCoef, minVal, maxVal, distTh;

		Model(po::variables_map vm) {
		    nParticles = vm["nParticles"].as<long>();
		    maxTrials = vm["maxTrials"].as<long>();
		    maxDistTry = vm["maxDistTry"].as<long>();

		    exploreCoef = vm["exploreCoef"].as<double>();
		    depthTh = vm["depthTh"].as<double>();
		    distTh = vm["distTh"].as<double>();

		    discount = vm["discount"].as<double>();
        }

		inline double getDiscount() { return discount; }
		inline long getMaxTrials() { return maxTrials; }
		inline double getDepthTh() { return depthTh; }
		inline long getNParticles() { return nParticles; }
		inline long getNActions() { return nActions; }
		inline long getNStVars() { return nStVars; }
		inline double getExploreCoef() { return exploreCoef; }
		
		/***** Start virtual functions *****/
		virtual void sampleAnInitState(StateVals& tmpStVals)=0;		
		virtual void solveHeuristic(StateVals &s, double *qVal)=0;
		virtual bool getNextState(StateVals &sVals, long actIdx, StateVals &nxtSVals, ObsVals &obs)=0;
		virtual double getReward(StateVals &sVals)=0;
		virtual double getReward(StateVals &sVals, long actId)=0;
		virtual double getNextStateNRew(StateVals &currStVals, long actId, ObsVals &obs, bool &isTerm)=0;
		virtual bool getNextState(StateVals &currStVals, long actIdx, double *immediateRew, StateVals &nxtSVals, ObsVals &obs)=0;
		virtual void setChanges(const char* chName, vector<long> &chTime)=0;
		virtual void update(long tCh, vector<StateVals> &affectedRange, vector<ChType> &typeOfChanges)=0;
		virtual double getDefaultVal()=0;
		virtual void getStatesSeeObs(long actId, ObsVals &obs, vector<StateVals> &partSt, map<int, StateVals> &partNxtSt)=0;
		virtual void getStatesSeeObs(ObsVals &obs, vector<StateVals> &posNxtSt)=0;
		virtual bool isTerm(StateVals &sVals)=0;
		virtual bool modifStSeq(vector<StateVals> &seqStVals, long startAffectedIdx, long endAffectedIdx,
				vector<StateVals> &modifStSeq, vector<long> &modifActSeq, vector<ObsVals> &modifObsSeq,
				vector<double> &modifRewSeq)=0;
		virtual void drawEnv(ostream &os)=0;
};

#endif
