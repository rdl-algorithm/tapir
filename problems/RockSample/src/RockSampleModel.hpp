#ifndef ROCKSAMPLEMODEL_HPP
#define ROCKSAMPLEMODEL_HPP

#include <cstddef>                      // for size_t
#include <cstdlib>                      // for abs

#include <algorithm>                    // for copy
#include <ios>                          // for dec, hex
#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for ostream, operator<<, basic_ostream, basic_ostream::operator<<, basic_ostream<>::__ostream_type
#include <string>                       // for string
#include <vector>                       // for vector

#include <boost/program_options.hpp>    // for program_options, variables_map

#include "ChangeType.hpp"               // for ChangeType
#include "Model.hpp"                    // for Model
#include "Observation.hpp"              // for Observation
#include "State.hpp"                    // for State

namespace po = boost::program_options;

struct Coords {
    long i;
    long j;
    Coords() :
                i(0),
                j(0) {
    }
    Coords(long i, long j) :
                i(i),
                j(j) {
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
inline bool operator!=(const Coords &lhs, const Coords &rhs) {
    return !(lhs == rhs);
}

class RockSampleModel: public Model {
public:
    RockSampleModel(po::variables_map vm);
    ~RockSampleModel() = default;
    RockSampleModel(const RockSampleModel&) = delete;
    RockSampleModel(RockSampleModel&&) = delete;
    RockSampleModel &operator=(const RockSampleModel&) = delete;
    RockSampleModel &operator=(RockSampleModel&&) = delete;

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
            os << std::dec;
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

    void dispState(VectorState &s, std::ostream &os) {
        os << Coords(s[0], s[1]) << " GOOD: {";
        std::vector<int> goodRocks;
        std::vector<int> badRocks;
        for (std::size_t i = 2; i < s.size(); i++) {
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

    void dispObs(Observation &o, std::ostream &os) {
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

    /***** Start implementation of Model's virtual methods *****/
    // Simple getters
    double getDiscount() {
        return discount;
    }
    unsigned long getNActions() {
        return nActions;
    }
    unsigned long getNObservations() {
        return nObservations;
    }
    unsigned long getNStVars() {
        return nStVars;
    }
    double getMinVal() {
        return minVal;
    }
    double getMaxVal() {
        return maxVal;
    }

    unsigned long getNParticles() {
        return nParticles;
    }
    long getMaxTrials() {
        return maxTrials;
    }
    double getDepthTh() {
        return depthTh;
    }
    double getExploreCoef() {
        return exploreCoef;
    }
    long getMaxDistTry() {
        return maxDistTry;
    }
    double getDistTh() {
        return distTh;
    }

    // Other virtual methods
    void sampleAnInitState(VectorState &sVals);
    bool isTerm(VectorState &sVals);
    void solveHeuristic(VectorState &s, double *qVal);
    double getDefaultVal();

    bool getNextState(VectorState &sVals, unsigned long actIdx, double *immediateRew,
            VectorState &nxtSVals, Observation &obs);
    double getReward(VectorState &sVals);
    double getReward(VectorState &sVals, unsigned long actId);

    void getStatesSeeObs(unsigned long actId, Observation &obs,
            std::vector<VectorState> &partSt, std::vector<VectorState> &partNxtSt);
    void getStatesSeeObs(unsigned long actId, Observation &obs,
            std::vector<VectorState> &partNxtSt);

    void getChangeTimes(const char *chName, std::vector<long> &chTime);
    void update(long tCh, std::vector<VectorState> &affectedRange,
            std::vector<ChangeType> &typeOfChanges);
    bool modifStSeq(std::vector<VectorState> &seqStVals, long startAffectedIdx,
            long endAffectedIdx, std::vector<VectorState> &modifStSeq,
            std::vector<long> &modifActSeq,
            std::vector<Observation> &modifObsSeq,
            std::vector<double> &modifRewSeq);

    void drawEnv(std::ostream &os);
    void drawState(VectorState &s, std::ostream &os);

private:
    // Problem parameters.
    double discount;
    unsigned long nActions, nObservations, nStVars;
    double minVal, maxVal;

    // SBT parameters
    unsigned long nParticles;
    long maxTrials;
    double depthTh;
    double exploreCoef;

    long maxDistTry;
    double distTh;

    /** The number of state particles in the initial belief. */
    long nInitBel;
    /** A vector of all the states in the initial belief. */
    std::vector<VectorState> initBel;

    /**
     * Finds and counts the rocks on the map, and initialisese the required
     * data structures and variables.
     */
    void initialise();

    /** Generates a state uniformly at random. */
    void sampleStateUniform(VectorState &sVals);
    /** Generates the state of the rocks uniformly at random. */
    void sampleRocks(VectorState &sVals);
    /** Decodes rocks from an integer. */
    void decodeRocks(long val, VectorState &sVals);

    /**
     * Generates a next state for the given state and action;
     * returns true if the action was legal, and false if it was illegal.
     */
    bool makeNextState(VectorState &sVals, long actId, VectorState &nxtSVals);
    /** Generates an observation given a next state (i.e. after the action)
     * and an action.
     */
    int makeObs(VectorState &nxtSVals, long actId);

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

#endif /* ROCKSAMPLE_MODEL_HPP */
