#ifndef TAGMODEL_HPP_
#define TAGMODEL_HPP_

#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream
#include <string>                       // for string
#include <utility>                      // for pair
#include <vector>                       // for vector

#include <boost/program_options.hpp>    // for variables_map

#include "defs.hpp"                     // for RandomGenerator
#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions
#include "solver/Action.hpp"            // for Action
#include "solver/ChangeFlags.hpp"        // for ChangeFlags
#include "solver/Model.hpp"             // for Model::StepResult, Model
#include "solver/Observation.hpp"       // for Observation
#include "solver/State.hpp"

namespace po = boost::program_options;

namespace solver {
class StatePool;
} /* namespace solver */

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
    enum TagCellType : int {
        EMPTY = 0,
        WALL = -1
    };

    /** Either you've seen the opponent, or you haven't. 0*/
    enum TagObservation : int {
        UNSEEN = 0,
        SEEN = 1
    };

    std::string getName() {
        return "Tag";
    }

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

    bool isTerminal(solver::State const &state);
    double solveHeuristic(solver::State const &state);
    double getDefaultVal();

    /* --------------- Black box dynamics ----------------- */
    virtual std::unique_ptr<solver::State> generateNextState(
            solver::State const &state, solver::Action const &action);
    virtual std::unique_ptr<solver::Observation> generateObservation(
            solver::Action const &action, solver::State const &nextState);
    virtual double getReward(solver::State const &state,
                solver::Action const &action);
    virtual Model::StepResult generateStep(solver::State const &state,
            solver::Action const &action);

    std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::Action const &action,
            solver::Observation const &obs,
            std::vector<solver::State *> const &previousParticles);
    std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::Action const &action,
            solver::Observation const &obs);

    std::vector<long> loadChanges(char const *changeFilename);
    void update(long time, solver::StatePool *pool);

    void dispAct(solver::Action const &action, std::ostream &os);
    /** Displays a single cell of the map. */
    void dispCell(TagCellType cellType, std::ostream &os);
    void dispObs(solver::Observation const &obs, std::ostream &os);
    void drawEnv(std::ostream &os);
    void drawState(solver::State const &state, std::ostream &os);

  private:
    /** Initialises the required data structures and variables */
    void initialise();

    /** Generates a random empty grid cell. */
    GridPosition randomEmptyCell();

    /**
     * Generates a next state for the given state and action;
     * returns true if the action was legal, and false if it was illegal.
     */
    std::pair<std::unique_ptr<TagState>, bool> makeNextState(
            solver::State const &state, solver::Action const &action);
    /** Generates an observation given a next state (i.e. after the action)
     * and an action.
     */
    std::unique_ptr<solver::Observation> makeObservation(solver::Action const &action,
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

    /** The penalty for each movement. */
    double moveCost_;
    /** The reward for taggint the opponent. */
    double tagReward_;
    /** The penalty for failing a tag attempt. */
    double failedTagPenalty_;
    /** The probability that the opponent will stay still. */
    double opponentStayProbability_;

    /** The number of rows in the map. */
    long nRows_;
    /** The number of columns in the map. */
    long nCols_;

    /** The number of empty cells in the map. */
    long nEmptyCells_;

    /** The environment map in text form. */
    std::vector<std::string> mapText_;
    /** The environment map in vector form. */
    std::vector<std::vector<TagCellType>> envMap_;

    /** Represents a change in the Tag model. */
    struct TagChange {
        std::string changeType = "";
        double i0 = 0;
        double i1 = 0;
        double j0 = 0;
        double j1 = 0;
    };

    /** The changes (scheduled for simulation). */
    std::map<long, std::vector<TagChange>> changes_;

    // General problem parameters
    unsigned long nActions_, nObservations_, nStVars_;
    double minVal_, maxVal_;
};
} /* namespace tag */

#endif /* TAGMODEL_HPP_ */
