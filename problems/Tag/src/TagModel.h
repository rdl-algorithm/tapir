#ifndef TagModel_H
#define TagModel_H

#include <ostream>
#include <iomanip>
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

class TagModel: public Model {
public:
    TagModel(po::variables_map vm);
    virtual ~TagModel();

    /** Enumerates the possible actions */
    enum Action
        : int {
            NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3, TAG = 4
    };

    void dispAct(long actId, std::ostream &os) {
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

    void dispState(StateVals &s, std::ostream &os) {
        os << "ROBOT: " << decodeCoords(s[0]) << " OPPONENT: "
                << decodeCoords(s[1]);
        if (s[2] == TAGGED) {
            os << " TAGGED!";
        }
    }

    enum Obs
        : int {
            UNSEEN = 0, SEEN = 1
    };

    void dispObs(ObsVals &o, std::ostream &os) {
        os << decodeCoords(o[0]);
        if (o[1] == SEEN) {
            os << " SEEN!";
        }
    }

    /***** Start implementation of Model's virtual functions *****/

    void sampleAnInitState(StateVals &sVals);
    bool isTerm(StateVals &sVals);
    void solveHeuristic(StateVals &s, double *qVal);
    double getDefaultVal();

    bool getNextState(StateVals &sVals, long actId, double *immediateRew,
            StateVals &nxtSVals, ObsVals &obs);
    double getReward(StateVals &sVals);
    double getReward(StateVals &sVals, long actId);

    void getStatesSeeObs(long actId, ObsVals &obs,
            std::vector<StateVals> &partSt, std::vector<StateVals> &partNxtSt);
    void getStatesSeeObs(long actId, ObsVals &obs,
            std::vector<StateVals> &partNxtSt);

    void setChanges(const char *chName, std::vector<long> &chTime);
    void update(long tCh, std::vector<StateVals> &affectedRange,
            std::vector<Change> &typeOfChanges);
    bool modifStSeq(std::vector<StateVals> &seqStVals, long startAffectedIdx,
            long endAffectedIdx, std::vector<StateVals> &modifStSeq,
            std::vector<long> &modifActSeq, std::vector<ObsVals> &modifObsSeq,
            std::vector<double> &modifRewSeq);

    void drawEnv(std::ostream &os);

private:
    /** Initialises the required data structures and variables */
    void initialise();

    /** Generates an untagged state uniformly at random. */
    void sampleStateUniform(StateVals &sVals);

    /**
     * Generates a next state for the given state and action;
     * returns true if the action was legal, and false if it was illegal.
     */
    bool makeNextState(StateVals &sVals, long actId, StateVals &nxtSVals);
    /** Generates an observation given a next state (i.e. after the action)
     * and an action.
     */
    void makeObs(StateVals &nxtSVals, long actId, ObsVals &obsVals);
    /** Moves the opponent. */
    void moveOpponent(Coords &robotPos, Coords &opponentPos);
    /** Generates the distribution for the opponent's actions. */
    void makeOpponentActions(Coords &robotPos, Coords &opponentPos,
            std::vector<long> &actions);

    /** Gets the expected coordinates after taking the given action;
     *  this may result in invalid coordinates.
     */
    Coords getMovedPos(Coords &coords, long actId);
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
