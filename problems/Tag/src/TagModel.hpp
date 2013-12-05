#ifndef TAGMODEL_HPP
#define TAGMODEL_HPP

#include <cstdlib>                      // for abs

#include <iomanip>                      // for operator<<, setw
#include <ostream>                      // for operator<<, ostream, basic_ostream, basic_ostream::operator<<, basic_ostream<>::__ostream_type
#include <string>                       // for string
#include <vector>                       // for vector

#include <boost/program_options.hpp>    // for program_options, variables_map

#include "defs.hpp"                     // for RandomGenerator
#include "Action.hpp"                   // for Action
#include "ChangeType.hpp"               // for ChangeType
#include "Model.hpp"                    // for Model
#include "Observation.hpp"              // for Observation
#include "State.hpp"                    // for State

namespace po = boost::program_options;

class TagModel: public Model {
  public:
    TagModel(RandomGenerator *randGen, po::variables_map vm);
    ~TagModel() = default;
    TagModel(TagModel const &) = delete;
    TagModel(TagModel &&) = delete;
    TagModel &operator=(TagModel const &) = delete;
    TagModel &operator=(TagModel &&) = delete;

    /** Enumerates the possible actions */
    enum class TagAction : unsigned long {
        NORTH = 0,
        EAST = 1,
        SOUTH = 2,
        WEST = 3,
        TAG = 4
    };

    /** The cells are either empty or walls; empty cells are numbered
     * starting at 0
     */
    enum CellType : int {
        EMPTY = 0,
        WALL = -1
    };

    enum TaggedState
    : int {
        UNTAGGED = 0, TAGGED = 1
    };

    enum Obs
    : int {
        UNSEEN = 0, SEEN = 1
    };

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

    bool getNextState(VectorState &sVals, unsigned long actId, double *immediateRew,
                      VectorState &nxtSVals, Observation &obs);
    double getReward(VectorState &sVals);
    double getReward(VectorState &sVals, unsigned long actId);

    void getStatesSeeObs(unsigned long actId, Observation &obs,
                         std::vector<VectorState> &partSt, std::vector<VectorState> &partNxtSt);
    void getStatesSeeObs(unsigned long actId, Observation &obs,
                         std::vector<VectorState> &partNxtSt);

    void getChangeTimes(char const *chName, std::vector<long> &chTime);
    void update(long tCh, std::vector<VectorState> &affectedRange,
                std::vector<ChangeType> &typeOfChanges);
    bool modifStSeq(std::vector<VectorState> &seqStVals, long startAffectedIdx,
                    long endAffectedIdx, std::vector<VectorState> &modifStSeq,
                    std::vector<long> &modifActSeq,
                    std::vector<Observation> &modifObsSeq,
                    std::vector<double> &modifRewSeq);

    void dispAct(Action &action, std::ostream &os);
    void dispCell(CellType cellType, std::ostream &os);
    void dispObs(Observation const &obs, std::ostream &os);
    void drawEnv(std::ostream &os);
    void drawState(State const &state, std::ostream &os);

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
    void sampleStateUniform(VectorState &sVals);

    /**
     * Generates a next state for the given state and action;
     * returns true if the action was legal, and false if it was illegal.
     */
    bool makeNextState(VectorState &sVals, unsigned long actId, VectorState &nxtSVals);
    /** Generates an observation given a next state (i.e. after the action)
     * and an action.
     */
    void makeObs(VectorState &nxtSVals, unsigned long actId, Observation &obsVals);
    /** Moves the opponent. */
    void moveOpponent(GridPosition &robotPos, GridPosition &opponentPos);
    /** Generates the distribution for the opponent's actions. */
    void makeOpponentActions(GridPosition &robotPos, GridPosition &opponentPos,
                             std::vector<long> &actions);

    /** Gets the expected coordinates after taking the given action;
     *  this may result in invalid coordinates.
     */
    GridPosition getMovedPos(GridPosition &GridPosition, unsigned long actId);
    /** Returns true iff the given GridPosition form a valid position. */
    bool isValid(GridPosition &sVals);

    /** Encodes the coordinates as an integer. */
    long encodeGridPosition(GridPosition c);
    /** Decodes the coordinates from an integer. */
    GridPosition decodeGridPosition(long code);

    /** The number of rows in the map. */
    long nRows;
    /** The number of columns in the map. */
    long nCols;

    /** The number of empty cells in the map. */
    long nEmptyCells;
    /** The empty cells, numbered; */
    std::vector<GridPosition> emptyCells;

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
