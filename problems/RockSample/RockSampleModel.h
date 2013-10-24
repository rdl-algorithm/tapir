#ifndef RockSampleModel_H
#define RockSampleModel_H

#include <ostream>
#include <vector>
#include <map>
#include <string>

#include <cstdlib>

#include "Model.h"
#include "GlobalResources.h"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

struct Coords {
    long i;
    long j;
    Coords() : i(0), j(0) {}
    Coords(long i, long j) : i(i), j(j) {}

    double distance(Coords other) {
        return std::abs(i - other.i) + std::abs(j - other.j);
    }
};

class RockSampleModel : public Model {
	public:
		RockSampleModel(po::variables_map vm);
		~RockSampleModel();

		/***** Start implementation of Model's virtual functions *****/

        /**
         * Samples an initial state from the belief vector.
         */
		void sampleAnInitState(StateVals& tmpStVals);
		/** ? */
		bool getNextState(StateVals &sVals, long actIdx, StateVals &nxtSVals,
		        ObsVals &obs);
        /**
        * Approximates the q-value of a state.
        */
		void solveHeuristic(StateVals &s, double *qVal);
		double getReward(StateVals &sVals);
		double getReward(StateVals &sVals, long actId);
		double getNextStateNRew(StateVals &currStVals, long actId, ObsVals &obs,
		        bool &isTerm);
		bool getNextState(StateVals &currStVals, long actIdx,
		        double *immediateRew, StateVals &nxtSVals, ObsVals &obs);
		void setChanges(const char* chName, std::vector<long> &chTime);
		void update(long tCh, std::vector<StateVals> &affectedRange,
		        std::vector<ChType> &typeOfChanges);
		double getDefaultVal();
		void getStatesSeeObs(long actId, ObsVals &obs,
		        std::vector<StateVals> &partSt,
		        std::map<int, StateVals> &partNxtSt);
		void getStatesSeeObs(ObsVals &obs, std::vector<StateVals> &posNxtSt);
		bool isTerm(StateVals &sVals);
		bool modifStSeq(std::vector<StateVals> &seqStVals,
		        long startAffectedIdx, long endAffectedIdx,
				std::vector<StateVals> &modifStSeq,
				std::vector<long> &modifActSeq,
				std::vector<ObsVals> &modifObsSeq,
				std::vector<double> &modifRewSeq);
		void drawEnv(std::ostream &os);

	private:
	    /** Initialises the required parameters and data structures/ */
	    void initialise();

	    /**
	     * Enumerates the possible actions. Note that there are actually
	     * multiple check actions; Check-i is represented by CHECK+i,
	     * where i is the rock number from 0..k-1 and k is the number
	     * of rocks.
	     */
	    enum RockSampleAction {
	        NORTH=0,
	        EAST=1,
	        SOUTH=2,
	        WEST=3,
	        SAMPLE=4,
	        CHECK=5
        };

        /**
         * There are only two possible observations - the rock
         * is either good or bad. Note that observations are
         * only meaningful when the action taken was SAMPLE;
         * they are meaningless otherwise.
         */
        enum RockSampleObservation {
            GOOD = 0,
            BAD = 1
        };

        /** The map is square, mapSize * mapSize */
        long mapSize;
        /** The number of rocks on the map. */
        long nRocks;
        /** The starting position. */
        Coords startPos;
        /** The coordinates of the rocks. */
        std::vector<Coords> rockCoords;

        /** The reward for sampling a good rock. */
        double goodRockReward;
        /** The penalty for sampling a bad rock. */
        double badRockPenalty;
        /** The reward for exiting the map. */
        double exitReward;
        /** The penalty for an illegal move. */
        double illegalMovePenalty;

        /** The half efficiency distance d0 */
        double halfEfficiencyDistance;

        /** The environment map. */
        std::vector<std::string> envMap;
};

#endif
