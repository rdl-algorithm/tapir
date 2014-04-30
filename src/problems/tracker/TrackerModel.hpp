#ifndef TRACKERMODEL_HPP_
#define TRACKERMODEL_HPP_

#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream
#include <string>                       // for string
#include <utility>                      // for pair
#include <vector>                       // for vector

#include <boost/program_options.hpp>    // for variables_map

#include "global.hpp"                     // for RandomGenerator
#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions

#include "solver/changes/ChangeFlags.hpp"        // for ChangeFlags
#include "solver/abstract-problem/Model.hpp"             // for Model::StepResult, Model
#include "solver/abstract-problem/TransitionParameters.hpp"
#include "solver/abstract-problem/Action.hpp"            // for Action
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/abstract-problem/State.hpp"

#include "solver/mappings/enumerated_actions.hpp"
#include "solver/mappings/discrete_observations.hpp"

#include "TrackerAction.hpp"

namespace po = boost::program_options;

namespace solver {
class StatePool;
} /* namespace solver */

namespace tracker {

/** The cells are either empty or walls; empty cells are numbered
 * starting at 0
 */
enum TrackerCellType : int {
    EMPTY = 0,
    WALL = -1
};

class TrackerObervation;
class TrackerState;

class TrackerModel: virtual public ModelWithProgramOptions,
        virtual public solver::ModelWithEnumeratedActions,
        virtual public solver::ModelWithDiscreteObservations {
    friend class TrackerObservation;

  public:
    TrackerModel(RandomGenerator *randGen, po::variables_map vm, std::vector<std::vector<TrackerCellType>> envMap = std::vector<std::vector<TrackerCellType>>());
    ~TrackerModel() = default;
    TrackerModel(TrackerModel const &) = delete;
    TrackerModel(TrackerModel &&) = delete;
    TrackerModel &operator=(TrackerModel const &) = delete;
    TrackerModel &operator=(TrackerModel &&) = delete;

    std::string getName() override {
        return "Tracker";
    }

    /***** Start implementation of Model's virtual methods *****/
    // Simple getters
    long getNumberOfStateVariables() override {
        return nStVars_;
    }
    double getMinVal() override {
        return minVal_;
    }
    double getMaxVal() override {
        return maxVal_;
    }
    double getDefaultVal() override {
        return 0;
    }

    // Other virtual methods
    std::unique_ptr<solver::State> sampleAnInitState() override;
    /** Generates an untrackerged state uniformly at random. */
    std::unique_ptr<solver::State> sampleStateUniform() override;

    bool isTerminal(solver::State const &state) override;
    double getHeuristicValue(solver::State const &state) override;

    /* --------------- Black box dynamics ----------------- */
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
                solver::State const */*nextState*/) override;

    virtual Model::StepResult generateStep(solver::State const &state,
            solver::Action const &action) override;


    std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::BeliefNode *previousBelief,
            solver::Action const &action,
            solver::Observation const &obs,
            std::vector<solver::State const *>
                const &previousParticles) override;
    std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::BeliefNode *previousBelief,
            solver::Action const &action,
            solver::Observation const &obs) override;

    std::vector<long> loadChanges(char const *changeFilename) override;
    void update(long time, solver::StatePool *pool) override;

    /** Displays a single cell of the map. */
    void dispCell(TrackerCellType cellType, std::ostream &os);
    void drawEnv(std::ostream &os) override;
    void drawSimulationState(solver::BeliefNode *belief,
            solver::State const &state,
            std::ostream &os) override;

    virtual std::vector<std::unique_ptr<solver::DiscretizedPoint>> getAllActionsInOrder();

  private:
    /** Initialises the required data structures and variables */
    void initialize();

    /** Generates a random empty grid cell. */
    GridPosition randomEmptyCell();

    /**
     * Generates a next state for the given state and action;
     * returns true if the action was legal, and false if it was illegal.
     */
    std::pair<std::unique_ptr<TrackerState>, bool> makeNextState(
            solver::State const &state, solver::Action const &action);
    /** Generates an observation given a next state (i.e. after the action)
     * and an action.
     */
    std::unique_ptr<solver::Observation> makeObservation(
            solver::Action const &action, TrackerState const &nextState);
    /** Generates the distribution for the target's actions. */
    std::vector<ActionType> makeTargetActions();

    /** Gets the expected coordinates after taking the given action;
     *  this may result in invalid coordinates.
     */
    GridPosition getNewPos(GridPosition const &position, int yaw, ActionType action);
    int getNewYaw(int yaw, ActionType action);
    /** Returns true iff the given GridPosition form a valid position. */
    bool isValid(GridPosition const &pos);

    /** The penalty for each movement. */
    double moveCost_;
    /** The reward for trackergint the target. */
    double trackerReward_;
    /** The penalty for failing a tracker attempt. */
    double failedTrackerPenalty_;
    /** The probability that the target will stay still. */
    double targetStayProbability_;

    /** The number of rows in the map. */
    long nRows_;
    /** The number of columns in the map. */
    long nCols_;

    /** The environment map in text form. */
    std::vector<std::string> mapText_;
    /** The environment map in vector form. */
    std::vector<std::vector<TrackerCellType>> envMap_;

    /** Represents a change in the Tracker model. */
    struct TrackerChange {
        std::string changeType = "";
        double i0 = 0;
        double i1 = 0;
        double j0 = 0;
        double j1 = 0;
    };

    /** The changes (scheduled for simulation). */
    std::map<long, std::vector<TrackerChange>> changes_;

    // General problem parameters
    long nActions_, nStVars_;
    double minVal_, maxVal_;
};
} /* namespace tracker */

#endif /* TRACKERMODEL_HPP_ */
