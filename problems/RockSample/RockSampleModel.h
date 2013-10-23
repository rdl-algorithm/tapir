#ifndef RockSampleModel_H
#define RockSampleModel_H

#include <vector>
#include <map>
#include <string>
#include "Model.h"
#include "GlobalResources.h"
using namespace std;

class RockSampleModel : public Model {
	public:
		RockSampleModel(const char* mapFName, const char *paramFName);
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
		
	private:
	    enum Action {
	        NORTH=0, 
	        EAST=1, 
	        SOUTH=2, 
	        WEST=3,
	        SAMPLE=4,
	        CHECK=5
	    };

	    enum Observation {
	        NONE=0,
	        GOOD=1,
	        BAD=2
	    };
};

#endif
