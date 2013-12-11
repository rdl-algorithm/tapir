#ifndef TAGMODEL_HPP
#define TAGMODEL_HPP

#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream
#include <string>                       // for string
#include <utility>                      // for pair
#include <vector>                       // for vector

#include <boost/program_options.hpp>    // for program_options, variables_map

#include "defs.hpp"                     // for RandomGenerator
#include "problems/GridPosition.hpp"    // for GridPosition
#include "solver/Action.hpp"            // for Action
#include "solver/ChangeType.hpp"        // for ChangeType
#include "solver/Model.hpp"             // for Model, Model::StepResult
#include "solver/Observation.hpp"       // for Observation

class State;
class TagState;

namespace po = boost::program_options;

class TagModel : public Model {
  public:
    TagModel(RandomGenerator *randGen, po::variables_map vm);
    ~TagModel() = default;
    TagModel(TagModel const &) = delete;
    TagModel(TagModel &&) = delete;
    TagModel &operator=(TagModel const &) = delete;
    TagModel &operator=(TagModel &&) = delete;

    /** Enumerates the possible actions */
    enum TagAction : long {
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

    enum TagObservation : int {
        UNSEEN = 0,
        SEEN = 1
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
    std::unique_ptr<State> sampleAnInitState();
    /** Generates an untagged state uniformly at random. */
    std::unique_ptr<State> sampleStateUniform();

    bool isTerm(State const &state);
    double solveHeuristic(State const &state);
    double getDefaultVal();

    Model::StepResult generateStep(State const &state, Action const &action);
    double getReward(State const &state);
    double getReward(State const &state, Action const &action);

    std::vector<std::unique_ptr<State>> generateParticles(Action const &action,
            Observation const &obs,
            std::vector<State *> const &previousParticles);
    std::vector<std::unique_ptr<State>> generateParticles(Action const &action,
            Observation const &obs);

    std::vector<long> loadChanges(char const *changeFilename);
    void update(long time, std::vector<std::unique_ptr<State>> *affectedRange,
            std::vector<ChangeType> *typeOfChanges);

    bool modifStSeq(std::vector<State const *> const &states,
            long startAffectedIdx, long endAffectedIdx,
            std::vector<std::unique_ptr<State>> *modifStSeq,
            std::vector<Action> *modifActSeq,
            std::vector<Observation> *modifObsSeq,
            std::vector<double> *modifRewSeq);

    void dispAct(Action const &action, std::ostream &os);
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

    /**
     * Generates a next state for the given state and action;
     * returns true if the action was legal, and false if it was illegal.
     */
    std::pair<std::unique_ptr<TagState>, bool> makeNextState(
            State const &state, Action const &action);
    /** Generates an observation given a next state (i.e. after the action)
     * and an action.
     */
    Observation makeObs(Action const &action, TagState const &state);
    /** Moves the opponent. */
    GridPosition getMovedOpponentPos(GridPosition const &robotPos,
            GridPosition const &opponentPos);
    /** Generates the distribution for the opponent's actions. */
    std::vector<TagAction> makeOpponentActions(GridPosition const &robotPos,
            GridPosition const &opponentPos);

    /** Gets the expected coordinates after taking the given action;
     *  this may result in invalid coordinates.
     */
    GridPosition getMovedPos(GridPosition const &position,
            Action const &action);
    /** Returns true iff the given GridPosition form a valid position. */
    bool isValid(GridPosition const &pos);

    /** Encodes the coordinates as an integer. */
    long encodeGridPosition(GridPosition pos);
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
    std::vector<std::vector<CellType>> envMap;
};

#endif
