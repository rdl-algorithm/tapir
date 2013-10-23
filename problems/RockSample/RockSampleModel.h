#ifndef RockSampleModel_H
#define RockSampleModel_H

#include <vector>
#include <map>
#include <string>
#include "Model.h"
#include "GlobalResources.h"
#include "StRoadmap.h"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace std;

class RockSampleModel : public Model {
	public:
		RockSampleModel(po::variables_map vm);
		~RockSampleModel();
		
		/***** Start implementation of Model's virtual functions *****/
		void sampleAnInitState(StateVals& tmpStVals);	
		bool getNextState(StateVals &sVals, long actIdx, StateVals &nxtSVals, ObsVals &obs);
		void solveHeuristic(StateVals &s, double *qVal);
		double getReward(StateVals &sVals);
		double getReward(StateVals &sVals, long actId);
		double getNextStateNRew(StateVals &currStVals, long actId, ObsVals &obs, bool &isTerm);
		bool getNextState(StateVals &currStVals, long actIdx, double *immediateRew, StateVals &nxtSVals, ObsVals &obs);
		void setChanges(const char* chName, vector<long> &chTime);
		void update(long tCh, vector<StateVals> &affectedRange, vector<ChType> &typeOfChanges);
		double getDefaultVal();
		void getStatesSeeObs(long actId, ObsVals &obs, vector<StateVals> &partSt, map<int, StateVals> &partNxtSt);
		void getStatesSeeObs(ObsVals &obs, vector<StateVals> &posNxtSt);
		bool isTerm(StateVals &sVals);
		bool modifStSeq(vector<StateVals> &seqStVals, long startAffectedIdx, long endAffectedIdx,
				vector<StateVals> &modifStSeq, vector<long> &modifActSeq, vector<ObsVals> &modifObsSeq,
				vector<double> &modifRewSeq);
		void drawEnv(ostream &os);
		
		// Additional initialisation.
		void setInitObsGoal();
		
	private:
		enum { EAST=0, NORTH=1, SOUTH=2, NORTHEAST=3, SOUTHEAST=4 };
		
		long nX, nY, nObservations, nGoals, nRocks;
		double goalReward, crashPenalty, moveCost, moveDiagCost;
		double ctrlCorrectProb, ctrlErrProb1;
		double rolloutExploreTh;
		vector<string> envMap;
		vector<StateVals> goals;
		vector<StateVals> rocks;
		vector<StateVals> allObservations;
		map< long, map<long, short> > cellType;		// 0: usual, 1: goals, 2: rocks, 3: observation, 4: spc. reward, 5: obstacle.
		short nSpcRew;
		vector<double> spcRew;
		map< long, vector<string> > changes;
		vector<StateVals> obstacleRegion;

		StRoadmap *roadmap;
		long nTryCon, maxDistCon, nVerts;
		
		//double getExpDist(StateVals &s, long firstAct);
		double getDist(StateVals &s1, StateVals &s2);
		void getNextState(StateVals &s, long actId, StateVals &sp);
		void inObsRegion(StateVals &st, ObsVals &obs);
		double getDistToNearestGoal(StateVals &st);
		double getDistToNearestObs(StateVals &st, StateVals &nxtSt);
		bool inGoal(StateVals &st);
		bool inRock(StateVals &st);
		void getReachableSt(StateVals &s, long actId, vector<StateVals> &nxtS);
		vector<StateVals>::iterator getIterator(vector<StateVals> &vecStVals, long x, long y);

		int findCollision(StateVals &s);

};

#endif
