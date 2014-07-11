/** @file HomecareModel.hpp
 *
 * Contains HomecareModel, which implements the core Model interface for the Homecare POMDP.
 */
#ifndef HOMECAREMODEL_HPP_
#define HOMECAREMODEL_HPP_

#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream
#include <string>                       // for string
#include <utility>                      // for pair
#include <vector>                       // for vector

#include "global.hpp"                     // for RandomGenerator
#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions

#include "solver/abstract-problem/Model.hpp"             // for Model::StepResult, Model
#include "solver/abstract-problem/ModelChange.hpp"             // for ModelChange
#include "solver/abstract-problem/TransitionParameters.hpp"
#include "solver/abstract-problem/Action.hpp"            // for Action
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/abstract-problem/State.hpp"

#include "solver/mappings/actions/enumerated_actions.hpp"
#include "solver/mappings/observations/discrete_observations.hpp"

#include "HomecareAction.hpp"
#include "HomecareOptions.hpp"

namespace solver {
class StatePool;
} /* namespace solver */

/** A namespace to hold the various classes used for the Homecare POMDP model. */
namespace homecare {
class HomecareObervation;
class HomecareState;

/** Represents a change in the Homecare model. */
struct HomecareChange : public solver::ModelChange {
    /** The change type for this change:
     * - "Add W" to add new "washroom" zone, area where target has higher probability
     *   of not moving      
     */
    std::string changeType = "";
    /** The first row number where the change applies. */
    long i0 = 0;
    /** The last row number where the change applies. */
    long i1 = 0;
    /** The first column number where the change applies. */
    long j0 = 0;
    /** The last column number where the change applies. */
    long j1 = 0;
};

/** The implementation of the Model interface for the Homecare POMDP.
 *
 * See this paper http://robotics.itee.uq.edu.au/~hannakur/dokuwiki/papers/isrr13_abt.pdf
 * for a description of the Homecare problem.
 *
 * This class inherits from shared::ModelWithProgramOptions in order to use custom text-parsing
 * functionality to select many of the core ABT parameters, allowing the configuration options
 * to be changed easily via the configuration interface without having to recompile the code.
 */
class HomecareModel: public shared::ModelWithProgramOptions {
    friend class HomecareObservation;

  public:
    /** Constructs a new HomecareModel instance with the given random number engine, and the given set
     * of configuration options.
     */
    HomecareModel(RandomGenerator *randGen, std::unique_ptr<HomecareOptions> options);

    ~HomecareModel() = default;
    _NO_COPY_OR_MOVE(HomecareModel);

    /** An enumeration for the different types of path cells for Homecare - these describe the
     * different paths the Homecare target can take through the map, as well as the walls in the
     * map.
     */
    enum class HomecarePathCell : int {
        EMPTY = 0,
        UP = 1,
        RIGHT = 2,
        DOWN = 3,
        LEFT = 4,
        UP_OR_RIGHT = 5,
        DOWN_OR_RIGHT = 6,
        DOWN_OR_LEFT = 7,
        UP_OR_LEFT = 8,
        WALL = -1
    };

    /** An enumeration for the key cell types for Homecare - this includes the starting positions
     * and washrooms, which are places where the target is likely to spend more time.
     */
    enum class HomecareTypeCell : int {
        OTHER = 0,
        TARGET_START = 1,
        WASHROOM = 2,
        START = 3,
    };

    /* ---------- Custom getters for extra functionality  ---------- */
    /** Returns the number of rows in the map for this HomecareModel instance. */
    long getNRows() const {
        return nRows_;
    }
    /** Returns the number of columns in the map for this HomecareModel instance. */
    long getNCols() const {
        return nCols_;
    }

    /* --------------- The model interface proper ----------------- */
    std::unique_ptr<solver::State> sampleAnInitState() override;
    std::unique_ptr<solver::State> sampleStateUninformed() override;
    bool isTerminal(solver::State const &state) override;

    /* -------------------- Black box dynamics ---------------------- */
    virtual std::unique_ptr<solver::State> generateNextState(
            solver::State const &state,
            solver::Action const &action,
            solver::TransitionParameters const */*tp*/) override;
    virtual std::unique_ptr<solver::Observation> generateObservation(
            solver::State const */*state*/,
            solver::Action const &action,
            solver::TransitionParameters const */*tp*/,
            solver::State const &nextState) override;
    virtual double generateReward(
                solver::State const &state,
                solver::Action const &action,
                solver::TransitionParameters const */*tp*/,
                solver::State const *nextState) override;
    virtual Model::StepResult generateStep(solver::State const &state,
            solver::Action const &action) override;


    /* -------------- Methods for handling model changes ---------------- */
    virtual void applyChanges(std::vector<std::unique_ptr<solver::ModelChange>> const &changes,
             solver::Solver *solver) override;


    /* ------------ Methods for handling particle depletion -------------- */
    /** Generates particles for Homecare using a particle filter from the previous belief.
      *
      * For each previous particle, possible next states are calculated based on consistency with
      * the given action and observation. These next states are then added to the output vector
      * in accordance with their probability of having been generated.
      */
    virtual std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::BeliefNode *previousBelief,
            solver::Action const &action,
            solver::Observation const &obs,
            long nParticles,
            std::vector<solver::State const *> const &previousParticles) override;

    /** Generates particles for Homecare according to an uninformed prior.
     *
     * Previous states are sampled uniformly at random, a single step is generated, and only states
     * consistent with the action and observation are kept.
     */
    virtual std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::BeliefNode *previousBelief,
            solver::Action const &action,
            solver::Observation const &obs,
            long nParticles) override;


    /* --------------- Pretty printing methods ----------------- */
    /** Displays a single cell of the map. */
    //virtual void dispCell(HomecareCellType cellType, std::ostream &os);
    virtual void drawEnv(std::ostream &os) override;
    virtual void drawSimulationState(solver::BeliefNode const *belief,
            solver::State const &state,
            std::ostream &os) override;


    /* ---------------------- Basic customizations  ---------------------- */
    virtual double getDefaultHeuristicValue(solver::HistoryEntry const *entry,
                solver::State const *state, solver::HistoricalData const *data) override;

    /* ------- Customization of more complex solver functionality  --------- */
    /** Returns all of the actions available for the Homecare POMDP, in the order of their enumeration
     * (as specified by homecare::ActionType).
     */
    virtual std::vector<std::unique_ptr<solver::DiscretizedPoint>> getAllActionsInOrder();
    virtual std::unique_ptr<solver::ActionPool> createActionPool(solver::Solver *solver) override;

    virtual std::unique_ptr<solver::Serializer> createSerializer(solver::Solver *solver) override;

  private:

    /** Read map text from file */
    std::vector<std::string> readMapText(std::string filename);
    /** Initialises the required data structures and variables for this model. */
    void initialize();

    /** Generates random cell. */
    GridPosition randomTCell();
    GridPosition randomSCell();
    GridPosition randomPCell();
    GridPosition randomNotWallCell();

    /**
     * Generates a next state for the given state and action;
     * returns true if the action was legal, and false if it was illegal.
     */
    std::pair<std::unique_ptr<HomecareState>, bool> makeNextState(
            solver::State const &state, solver::Action const &action);
    /** Generates an observation given a next state (i.e. after the action)
     * and an action.
     */
    std::unique_ptr<solver::Observation> makeObservation(HomecareState const &nextState);

    /** Samples a reading from region sensors */
    int sampleObservationRegion(HomecareState const &state);

    /** Generates vector of valid cells target can move to*/
    std::vector<GridPosition> validMovedTargetPositions(
            GridPosition const &targetPos);
    /** Generates distribution after taking action */
    std::unordered_map<GridPosition, double> getNextRobotPositionDistribution(
            GridPosition const &robotPos, ActionType action);
    std::unordered_map<GridPosition, double> getNextTargetPositionDistribution(
            GridPosition const &targetPos, bool call);
    std::unordered_map<bool, double> getNextCallDistribution(
            GridPosition const &robotPos, GridPosition const &targetPos, bool call);


    /** Moves the robot/target. */
    std::pair<GridPosition, bool> sampleMovedRobotPosition(
            GridPosition const &robotPos, ActionType action);
    GridPosition sampleMovedTargetPosition(GridPosition const &targetPos);

    /** Gets the expected coordinates after taking the given action;
     *  this may result in invalid coordinates.
     */
    std::pair<GridPosition, bool> getMovedPos(GridPosition const &position, ActionType action);
    /** Returns true iff the given GridPosition represents a valid square that an agent could be
     * in - that is, the square must be empty, and within the bounds of the map.
     */
    bool isValid(GridPosition const &pos);
    /** E.g. returns NORTH if given NORTH_EAST */
    ActionType shiftAntiClockwise(ActionType action);
    /** E.g. returns NORTH_EAST if given NORTH */
    ActionType shiftClockwise(ActionType action);

    bool updateCall(GridPosition robotPos, GridPosition targetPos, bool call);

    HomecareOptions *options_;

    /** The penalty for each movement. */
    double moveCost_;
    double diaMoveCost_;
    /** The reward for reaching the target when it needs help */
    double helpReward_;
    /** The probability that the target stays still on a W tile */
    double targetWStayProbability_;
    /** The probability that the target will stay still. */
    double targetStayProbability_;
    /** The probability for correct robot motion */
    double moveAccuracy_;
    /** The probability for correct localisation of target by region sensors */
    double regionSensorAccuracy_;
    /** The probability target will need help */
    double callProbability_;
    /** The probability target will still need help if it currently does */
    double continueCallProbability_;

    /** The number of rows in the map. */
    long nRows_;
    /** The number of columns in the map. */
    long nCols_;

    /** The environment map in text form. */
    std::vector<std::string> pathMapText_;
    std::vector<std::string> typeMapText_;
    /** The environment map in vector form. */
    std::vector<std::vector<HomecarePathCell>> pathMap_;
    std::vector<std::vector<HomecareTypeCell>> typeMap_;
    /** Vector of special cells. */
    std::vector<GridPosition> wCells_;
    std::vector<GridPosition> tCells_;
    std::vector<GridPosition> sCells_;
    std::vector<GridPosition> pCells_;

    /** The number of possible actions in the Homecare POMDP. */
    long nActions_;

};
} /* namespace homecare */

#endif /* HOMECAREMODEL_HPP_ */
