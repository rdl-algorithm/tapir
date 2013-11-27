#ifndef TAGMODEL_HPP
#define TAGMODEL_HPP

#include <cstdlib>

#include <iomanip>
#include <ostream>
#include <string>
#include <vector>

#include "ChangeType.hpp"
#include "Model.hpp"
#include "Observation.hpp"
#include "State.hpp"

#include <boost/program_options.hpp>
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

class TagModel: public Model {
public:
    TagModel(po::variables_map vm);
    ~TagModel() = default;
    TagModel(const TagModel&) = delete;
    TagModel(TagModel&) = delete;
    TagModel &operator=(const TagModel&) = delete;
    TagModel &operator=(TagModel&) = delete;

    /** Enumerates the possible actions */
    enum Action
        : int {
            NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3, TAG = 4
    };

    void dispAct(unsigned long actId, std::ostream &os) {
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
        case TAG:
            os << "TAG";
            break;
        }
    }

    /** The cells are either empty or walls; empty cells are numbered
     * starting at 0
     */
    enum CellType
        : int {
            EMPTY = 0, WALL = -1
    };

    void dispCell(int cellType, std::ostream &os) {
        if (cellType >= EMPTY) {
            os << std::setw(2);
            os << cellType;
            return;
        }
        switch (cellType) {
        case WALL:
            os << "XX";
            break;
        default:
            os << "ERROR-" << cellType;
            break;
        }
    }

    enum TaggedState
        : int {
            UNTAGGED = 0, TAGGED = 1
    };

    void dispState(State &s, std::ostream &os) {
        os << "ROBOT: " << decodeCoords(s.vals[0]) << " OPPONENT: "
                << decodeCoords(s.vals[1]);
        if (s.vals[2] == TAGGED) {
            os << " TAGGED!";
        }
    }

    enum Obs
        : int {
            UNSEEN = 0, SEEN = 1
    };

    void dispObs(Observation &o, std::ostream &os) {
        os << decodeCoords(o[0]);
        if (o[1] == SEEN) {
            os << " SEEN!";
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
    void sampleAnInitState(State &sVals);
    bool isTerm(State &sVals);
    void solveHeuristic(State &s, double *qVal);
    double getDefaultVal();

    bool getNextState(State &sVals, unsigned long actId, double *immediateRew,
            State &nxtSVals, Observation &obs);
    double getReward(State &sVals);
    double getReward(State &sVals, unsigned long actId);

    void getStatesSeeObs(unsigned long actId, Observation &obs,
            std::vector<State> &partSt, std::vector<State> &partNxtSt);
    void getStatesSeeObs(unsigned long actId, Observation &obs,
            std::vector<State> &partNxtSt);

    void setChanges(const char *chName, std::vector<long> &chTime);
    void update(long tCh, std::vector<State> &affectedRange,
            std::vector<ChangeType> &typeOfChanges);
    bool modifStSeq(std::vector<State> &seqStVals, long startAffectedIdx,
            long endAffectedIdx, std::vector<State> &modifStSeq,
            std::vector<long> &modifActSeq,
            std::vector<Observation> &modifObsSeq,
            std::vector<double> &modifRewSeq);

    void drawEnv(std::ostream &os);
    void drawState(State &s, std::ostream &os);

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

    /** Initialises the required data structures and variables */
    void initialise();

    /** Generates an untagged state uniformly at random. */
    void sampleStateUniform(State &sVals);

    /**
     * Generates a next state for the given state and action;
     * returns true if the action was legal, and false if it was illegal.
     */
    bool makeNextState(State &sVals, unsigned long actId, State &nxtSVals);
    /** Generates an observation given a next state (i.e. after the action)
     * and an action.
     */
    void makeObs(State &nxtSVals, unsigned long actId, Observation &obsVals);
    /** Moves the opponent. */
    void moveOpponent(Coords &robotPos, Coords &opponentPos);
    /** Generates the distribution for the opponent's actions. */
    void makeOpponentActions(Coords &robotPos, Coords &opponentPos,
            std::vector<long> &actions);

    /** Gets the expected coordinates after taking the given action;
     *  this may result in invalid coordinates.
     */
    Coords getMovedPos(Coords &coords, unsigned long actId);
    /** Returns true iff the given coords form a valid position. */
    bool isValid(Coords &sVals);

    /** Encodes the coordinates as an integer. */
    long encodeCoords(Coords c);
    /** Decodes the coordinates from an integer. */
    Coords decodeCoords(long code);

    /** The number of rows in the map. */
    long nRows;
    /** The number of columns in the map. */
    long nCols;

    /** The number of empty cells in the map. */
    long nEmptyCells;
    /** The empty cells, numbered; */
    std::vector<Coords> emptyCells;

    /** The penalty for each movement. */
    double moveCost;
    /** The reward for taggint the opponent. */
    double tagReward;
    /** The penalty for failing a tag attempt. */
    double failedTagPenalty;
    /** The probability that the opponent will stay still. */
    double opponentStayProbability;

    /** The environment map in text form. */
    std::vector<std::string> mapText;
    /** The environment map in vector form. */
    std::vector<std::vector<int> > envMap;
};

#endif
