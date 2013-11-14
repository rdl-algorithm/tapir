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

    double distance(Coords& other) {
        return std::abs(i - other.i) + std::abs(j - other.j);
    }
};

inline bool operator==(const Coords& lhs, const Coords& rhs) {
    return lhs.i == rhs.i && lhs.j == rhs.j;
}

class RockSampleModel : public Model {
	public:
        /**
	     * Enumerates the possible actions. Note that there are actually
	     * multiple check actions; Check-i is represented by CHECK+i,
	     * where i is the rock number from 0..k-1 and k is the number
	     * of rocks.
	     */
	    enum Action : int {
	        NORTH=0,
	        EAST=1,
	        SOUTH=2,
	        WEST=3,
	        SAMPLE=4,
	        CHECK=5
        };

        void dispAct(int actId, std::ostream& os) {
            if (actId >= CHECK) {
                os << "CHECK-" << actId-CHECK;
                return;
            }
            switch(actID) {
                case NORTH:
                    os << "NORTH";
                    break;
                case EAST:
                    os << "EAST";
                    break;
                case SOUTH:
                    os << "SOUTH";
                    break;
                case WEST:
                    os << "WEST";
                    break;
                case SAMPLE:
                    os << "SAMPLE";
                    break;
            }
        }

        /**
         * There are only two possible observations - the rock
         * is either good or bad. Note that observations are
         * only meaningful when the action taken was CHECK;
         * they are meaningless otherwise.
         */
        enum Obs : int {
            NONE = 0,
            BAD = 1,
            GOOD = 2
        };

        /**
         * Rocks are enumerated 0, 1, 2, ... ;
         * other cell types should be negative.
         */
        enum CellType : int {
            ROCK = 0,
            EMPTY = -1,
            GOAL = -2,
        };

        void dispCell(int cellType, ostream& os) {
            if (cellType >= ROCK) {
                os << std::hex << cellType - ROCK;
                return;
            }
            switch(cellType) {
                case EMPTY:
                    os << '.';
                    break;
                case GOAL:
                    os << 'G';
                    break;
            }
        }



		RockSampleModel(po::variables_map vm);
		~RockSampleModel();

		/***** Start implementation of Model's virtual functions *****/

		void sampleAnInitState(StateVals& tmpStVals);
		bool isTerm(StateVals &sVals);
		void solveHeuristic(StateVals &s, double *qVal);
		double getDefaultVal();

		bool getNextState(StateVals &sVals, long actIdx,
		        double *immediateRew, StateVals &nxtSVals, ObsVals &obs);
		double getReward(StateVals &sVals);
		double getReward(StateVals &sVals, long actId);

		void getStatesSeeObs(long actId, ObsVals &obs,
		        std::vector<StateVals> &partSt,
		        std::vector<StateVals> &partNxtSt);
		void getStatesSeeObs(long actId, ObsVals &obs,
		        std::vector<StateVals> &partNxtSt);

		void setChanges(const char* chName, std::vector<long> &chTime);
		void update(long tCh, std::vector<StateVals> &affectedRange,
		        std::vector<ChType> &typeOfChanges);
		bool modifStSeq(std::vector<StateVals> &seqStVals,
		        long startAffectedIdx, long endAffectedIdx,
				std::vector<StateVals> &modifStSeq,
				std::vector<long> &modifActSeq,
				std::vector<ObsVals> &modifObsSeq,
				std::vector<double> &modifRewSeq);

		void drawEnv(std::ostream &os);

    private:
        /** The number of state particles in the initial belief. */
        long nInitBel;
        /** A vector of all the states in the initial belief. */
        std::vector<StateVals> initBel;

        /**
        * Finds and counts the rocks on the map, and initialisese the required
        * data structures and variables.
        */
	    void initialise();

        /**
         * Generates a next state for the given state and action;
         * returns true if the action was legal, and false if it was illegal.
         */
        bool makeNextState(StateVals &sVals, long actId, StateVals &nxtSVals);
        /** Generates an observation given a current state and action. */
        int makeObs(StateVals &sVals, long actId);


        /** The number of rows in the map. */
        long nRows;
        /** The number of columns in the map. */
        long nCols;
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

        /** The environment map in text form. */
        std::vector<std::string> mapText;

        /** The environment map in vector form. */
        std::vector<std::vector<int> > envMap;
};

#endif
