#ifndef UnderwaterNavModifModel_H
#define UnderwaterNavModifModel_H

#include <ostream>
#include <vector>
#include <map>
#include <string>

#include "Model.h"
#include "GlobalResources.h"
#include "StRoadmap.h"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

class UnderwaterNavModifModel : public Model {
	public:
		UnderwaterNavModifModel(po::variables_map vm);
		~UnderwaterNavModifModel();
		
		/***** Start implementation of Model's virtual functions *****/
		void sampleAnInitState(StateVals& tmpStVals);	
		bool getNextState(StateVals &sVals, long actIdx, StateVals &nxtSVals, ObsVals &obs);
		void solveHeuristic(StateVals &s, double *qVal);
		double getReward(StateVals &sVals);
		double getReward(StateVals &sVals, long actId);
		double getNextStateNRew(StateVals &currStVals, long actId, ObsVals &obs, bool &isTerm);
		bool getNextState(StateVals &currStVals, long actIdx, double *immediateRew, StateVals &nxtSVals, ObsVals &obs);
		void setChanges(const char* chName, std::vector<long> &chTime);
		void update(long tCh, std::vector<StateVals> &affectedRange, std::vector<ChType> &typeOfChanges);
		double getDefaultVal();
		void getStatesSeeObs(long actId, ObsVals &obs, std::vector<StateVals> &partSt, std::map<int, StateVals> &partNxtSt);
		void getStatesSeeObs(ObsVals &obs, std::vector<StateVals> &posNxtSt);
		bool isTerm(StateVals &sVals);
		bool modifStSeq(std::vector<StateVals> &seqStVals, long startAffectedIdx, long endAffectedIdx,
				std::vector<StateVals> &modifStSeq, std::vector<long> &modifActSeq, std::vector<ObsVals> &modifObsSeq,
				std::vector<double> &modifRewSeq);
		void drawEnv(std::ostream &os);
		
		// Additional initialisation.
		void setInitObsGoal();
		
	private:
		enum { EAST=0, NORTH=1, SOUTH=2, NORTHEAST=3, SOUTHEAST=4 };
		
		long nX, nY, nGoals, nRocks;
		double goalReward, crashPenalty, moveCost, moveDiagCost;
		double ctrlCorrectProb, ctrlErrProb1;
		double rolloutExploreTh;
		std::vector<std::string> envMap;
		std::vector<StateVals> goals;
		std::vector<StateVals> rocks;
		std::vector<StateVals> allObservations;
		std::map< long, std::map<long, short> > cellType;		// 0: usual, 1: goals, 2: rocks, 3: observation, 4: spc. reward, 5: obstacle.
		short nSpcRew;
		std::vector<double> spcRew;
		std::map< long, std::vector<std::string> > changes;
		std::vector<StateVals> obstacleRegion;

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
		void getReachableSt(StateVals &s, long actId, std::vector<StateVals> &nxtS);
		std::vector<StateVals>::iterator getIterator(std::vector<StateVals> &vecStVals, long x, long y);

		int findCollision(StateVals &s);

};

#endif
