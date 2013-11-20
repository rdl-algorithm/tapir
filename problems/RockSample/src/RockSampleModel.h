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
    Coords() :
            i(0), j(0) {
    }
    Coords(long i, long j) :
            i(i), j(j) {
    }

    double distance(Coords &other) {
        return std::abs(i - other.i) + std::abs(j - other.j);
    }

};

inline std::ostream &operator<<(std::ostream &os, const Coords &obj) {
    os << "(" << obj.i << ", " << obj.j << ")";
    return os;
}
inline bool operator==(const Coords &lhs, const Coords &rhs) {
    return lhs.i == rhs.i && lhs.j == rhs.j;
}

class RockSampleModel: public Model {
public:
    RockSampleModel(po::variables_map vm);
    ~RockSampleModel();

    /**
     * Enumerates the possible actions. Note that there are actually
     * multiple check actions; Check-i is represented by CHECK+i,
     * where i is the rock number from 0..k-1 and k is the number
     * of rocks.
     */
    enum Action
        : long {
            NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3, SAMPLE = 4, CHECK = 5
    };

    void dispAct(unsigned long actId, std::ostream &os) {
        if (actId >= CHECK) {
            os << "CHECK-" << actId - CHECK;
            return;
        }
        switch (actId) {
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
        default:
            os << "ERROR-" << actId;
            break;
        }
    }

    /**
     * There are only two possible observations - the rock
     * is either good or bad. Note that observations are
     * only meaningful when the action taken was CHECK;
     * they are meaningless otherwise.
     */
    enum Obs
        : int {
            NONE = 0, BAD = 1, GOOD = 2
    };

    /**
     * Rocks are enumerated 0, 1, 2, ... ;
     * other cell types should be negative.
     */
    enum CellType
        : int {
            ROCK = 0, EMPTY = -1, GOAL = -2,
    };

    void dispCell(int cellType, std::ostream &os) {
        if (cellType >= ROCK) {
            os << std::hex << cellType - ROCK;
            return;
        }
        switch (cellType) {
        case EMPTY:
            os << '.';
            break;
        case GOAL:
            os << 'G';
            break;
        default:
            os << "ERROR-" << cellType;
            break;
        }
    }

    void dispState(StateVals &s, std::ostream &os) {
        os << Coords(s[0], s[1]) << " GOOD: {";
        std::vector<int> goodRocks;
        std::vector<int> badRocks;
        for (int i = 2; i < s.size(); i++) {
            if (s[i] == GOOD) {
                goodRocks.push_back(i - 2);
            } else {
                badRocks.push_back(i - 2);
            }
        }
        std::copy(goodRocks.begin(), goodRocks.end(),
                std::ostream_iterator<double>(os, " "));
        os << "}; BAD: {";
        std::copy(badRocks.begin(), badRocks.end(),
                std::ostream_iterator<double>(os, " "));
        os << "}";
    }

    void dispObs(ObsVals &o, std::ostream &os) {
        switch ((int) o[0]) {
        case NONE:
            os << "NONE";
            break;
        case GOOD:
            os << "GOOD";
            break;
        case BAD:
            os << "BAD";
            break;
        default:
            os << "ERROR-" << o[0];
            break;
        }
    }

    /***** Start implementation of Model's virtual functions *****/
    // Simple getters
    inline unsigned long getNActions() {
        return nActions;
    }
    inline unsigned long getNObservations() {
        return nObservations;
    }
    inline unsigned long getNStVars() {
        return nStVars;
    }
    inline double getMinVal() {
        return minVal;
    }
    inline double getMaxVal() {
        return maxVal;
    }

    void sampleAnInitState(StateVals &sVals);
    bool isTerm(StateVals &sVals);
    void solveHeuristic(StateVals &s, double *qVal);
    double getDefaultVal();

    bool getNextState(StateVals &sVals, unsigned long actIdx, double *immediateRew,
            StateVals &nxtSVals, ObsVals &obs);
    double getReward(StateVals &sVals);
    double getReward(StateVals &sVals, unsigned long actId);

    void getStatesSeeObs(unsigned long actId, ObsVals &obs,
            std::vector<StateVals> &partSt, std::vector<StateVals> &partNxtSt);
    void getStatesSeeObs(unsigned long actId, ObsVals &obs,
            std::vector<StateVals> &partNxtSt);

    void setChanges(const char *chName, std::vector<long> &chTime);
    void update(long tCh, std::vector<StateVals> &affectedRange,
            std::vector<Change> &typeOfChanges);
    bool modifStSeq(std::vector<StateVals> &seqStVals, unsigned long startAffectedIdx,
            unsigned long endAffectedIdx, std::vector<StateVals> &modifStSeq,
            std::vector<long> &modifActSeq, std::vector<ObsVals> &modifObsSeq,
            std::vector<double> &modifRewSeq);

    void drawEnv(std::ostream &os);

private:
    // Values for the required getters
    unsigned long nActions, nObservations, nStVars;
    double minVal, maxVal;

    /** The number of state particles in the initial belief. */
    long nInitBel;
    /** A vector of all the states in the initial belief. */
    std::vector<StateVals> initBel;

    /**
     * Finds and counts the rocks on the map, and initialisese the required
     * data structures and variables.
     */
    void initialise();

    /** Generates a state uniformly at random. */
    void sampleStateUniform(StateVals &sVals);
    /** Generates the state of the rocks uniformly at random. */
    void sampleRocks(StateVals &sVals);
    /** Decodes rocks from an integer. */
    void decodeRocks(long val, StateVals &sVals);

    /**
     * Generates a next state for the given state and action;
     * returns true if the action was legal, and false if it was illegal.
     */
    bool makeNextState(StateVals &sVals, long actId, StateVals &nxtSVals);
    /** Generates an observation given a next state (i.e. after the action)
     * and an action.
     */
    int makeObs(StateVals &nxtSVals, long actId);

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
