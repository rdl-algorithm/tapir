#ifndef TAGMODEL_HPP_
#define TAGMODEL_HPP_

#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream
#include <string>                       // for string
#include <utility>                      // for pair
#include <vector>                       // for vector

#include <boost/program_options.hpp>    // for variables_map

#include "defs.hpp"                     // for RandomGenerator
#include "problems/GridPosition.hpp"    // for GridPosition
#include "problems/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions
#include "solver/Action.hpp"            // for Action
#include "solver/ChangeType.hpp"        // for ChangeType
#include "solver/Model.hpp"             // for Model::StepResult, Model
#include "solver/Observation.hpp"       // for Observation

namespace solver {
class State;
} /* namespace solver */

namespace po = boost::program_options;

namespace tag {
class TagState;

class TagModel : public ModelWithProgramOptions {
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
    unsigned long getNActions() {
        return nActions_;
    }
    unsigned long getNObservations() {
        return nObservations_;
    }
    unsigned long getNStVars() {
        return nStVars_;
    }
    double getMinVal() {
        return minVal_;
    }
    double getMaxVal() {
        return maxVal_;
    }

    // Other virtual methods
    std::unique_ptr<solver::State> sampleAnInitState();
    /** Generates an untagged state uniformly at random. */
    std::unique_ptr<solver::State> sampleStateUniform();

    bool isTerm(solver::State const &state);
    double solveHeuristic(solver::State const &state);
    double getDefaultVal();

    solver::Model::StepResult generateStep(solver::State const &state,
            solver::Action const &action);
    double getReward(solver::State const &state);
    double getReward(solver::State const &state, solver::Action const &action);

    std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::Action const &action,
            solver::Observation const &obs,
            std::vector<solver::State *> const &previousParticles);
    std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::Action const &action,
            solver::Observation const &obs);

    std::vector<long> loadChanges(char const *changeFilename);
    void update(long time,
            std::vector<std::unique_ptr<solver::State>> *affectedRange,
            std::vector<solver::ChangeType> *typeOfChanges);

    bool modifStSeq(std::vector<solver::State const *> const &states,
            long startAffectedIdx, long endAffectedIdx,
            std::vector<std::unique_ptr<solver::State>> *modifStSeq,
            std::vector<solver::Action> *modifActSeq,
            std::vector<solver::Observation> *modifObsSeq,
            std::vector<double> *modifRewSeq);

    void dispAct(solver::Action const &action, std::ostream &os);
    void dispCell(CellType cellType, std::ostream &os);
    void dispObs(solver::Observation const &obs, std::ostream &os);
    void drawEnv(std::ostream &os);
    void drawState(solver::State const &state, std::ostream &os);

  private:
    // Problem parameters
    unsigned long nActions_, nObservations_, nStVars_;
    double minVal_, maxVal_;

    /** Initialises the required data structures and variables */
    void initialise();

    /**
     * Generates a next state for the given state and action;
     * returns true if the action was legal, and false if it was illegal.
     */
    std::pair<std::unique_ptr<TagState>, bool> makeNextState(
            solver::State const &state, solver::Action const &action);
    /** Generates an observation given a next state (i.e. after the action)
     * and an action.
     */
    solver::Observation makeObs(solver::Action const &action,
            TagState const &state);
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
            solver::Action const &action);
    /** Returns true iff the given GridPosition form a valid position. */
    bool isValid(GridPosition const &pos);

    /** Encodes the coordinates as an integer. */
    long encodeGridPosition(GridPosition pos);
    /** Decodes the coordinates from an integer. */
    GridPosition decodeGridPosition(long code);

    /** The number of rows in the map. */
    long nRows_;
    /** The number of columns in the map. */
    long nCols_;

    /** The number of empty cells in the map. */
    long nEmptyCells_;
    /** The empty cells, numbered; */
    std::vector<GridPosition> emptyCells_;

    /** The penalty for each movement. */
    double moveCost_;
    /** The reward for taggint the opponent. */
    double tagReward_;
    /** The penalty for failing a tag attempt. */
    double failedTagPenalty_;
    /** The probability that the opponent will stay still. */
    double opponentStayProbability_;

    /** The environment map in text form. */
    std::vector<std::string> mapText_;
    /** The environment map in vector form. */
    std::vector<std::vector<CellType>> envMap_;
};
} /* namespace tag */

#endif /* TAGMODEL_HPP_ */
