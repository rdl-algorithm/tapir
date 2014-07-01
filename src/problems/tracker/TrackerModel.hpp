#ifndef TRACKERMODEL_HPP_
#define TRACKERMODEL_HPP_

#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream
#include <string>                       // for string
#include <utility>                      // for pair
#include <vector>                       // for vector

#include "global.hpp"                     // for RandomGenerator
#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions

#include "solver/changes/ChangeFlags.hpp"        // for ChangeFlags
#include "solver/abstract-problem/Model.hpp"             // for Model::StepResult, Model
#include "solver/abstract-problem/ModelChange.hpp"             // for ModelChange
#include "solver/abstract-problem/TransitionParameters.hpp"
#include "solver/abstract-problem/Action.hpp"            // for Action
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/abstract-problem/State.hpp"

#include "solver/mappings/actions/enumerated_actions.hpp"
#include "solver/mappings/observations/discrete_observations.hpp"

#include "TrackerAction.hpp"
#include "TrackerOptions.hpp"

namespace solver {
class StatePool;
} /* namespace solver */

namespace tracker {
class TrackerObervation;
class TrackerState;

/** Represents a change in the Tracker model. */
struct TrackerChange : public solver::ModelChange {
    std::string changeType = "";
    long i0 = 0;
    long i1 = 0;
    long j0 = 0;
    long j1 = 0;

    TrackerChange(std::string argType, long argi0, long argi1,
    long argj0, long argj1) : 
        changeType(argType),
        i0(argi0),
        i1(argi1),
        j0(argj0),
        j1(argj1)
    {
    }
};

struct TargetState {
    GridPosition pos;
    int yaw;
    double probability;

    TargetState(GridPosition argPos, int argYaw, double argProb = 0) :
        pos(argPos),
        yaw(argYaw),
        probability(argProb)
    {
    }

    bool operator==(const TargetState &other) const {
        return pos == other.pos && yaw == other.yaw;
    }
};

struct TargetStateHash {
    inline std::size_t operator()(const TargetState &t) const {
        return t.pos.i * 111 + t.pos.j + t.yaw;
    }
};

class TrackerModel: public shared::ModelWithProgramOptions {
    friend class TrackerObservation;

  public:

    TrackerModel(RandomGenerator *randGen, std::unique_ptr<TrackerOptions> options);
    ~TrackerModel() = default;
    TrackerModel(TrackerModel const &) = delete;
    TrackerModel(TrackerModel &&) = delete;
    TrackerModel &operator=(TrackerModel const &) = delete;
    TrackerModel &operator=(TrackerModel &&) = delete;

    /** The cells are either empty or walls. */
    enum TrackerCellType : int {
        EMPTY = 0,
        WALL = -1
    };

    void setEnvMap(std::vector<std::vector<TrackerCellType>> envMap);
    void setPolicyZones(std::vector<GridPosition> zones, int currZone, double moveProbability);

    /* ---------- Custom getters for extra functionality  ---------- */
    long getNRows() const {
        return nRows_;
    }
    long getNCols() const {
        return nCols_;
    }

    /* --------------- The model interface proper ----------------- */
    // Other virtual methods
    std::unique_ptr<solver::State> sampleAnInitState() override;
    /** Generates an untrackerged state uniformly at random. */
    std::unique_ptr<solver::State> sampleStateUniform() override;
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
                solver::State const */*nextState*/) override;

    virtual Model::StepResult generateStep(solver::State const &state,
            solver::Action const &action) override;


    /* -------------- Methods for handling model changes ---------------- */
    virtual void applyChanges(std::vector<std::unique_ptr<solver::ModelChange>> const &changes,
             solver::Solver *solver) override;


    /* ------------ Methods for handling particle depletion -------------- */
    virtual std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::BeliefNode *previousBelief,
            solver::Action const &action,
            solver::Observation const &obs,
            long nParticles,
            std::vector<solver::State const *> const &previousParticles) override;
    virtual std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::BeliefNode *previousBelief,
            solver::Action const &action,
            solver::Observation const &obs,
            long nParticles) override;


    /* --------------- Pretty printing methods ----------------- */
    /** Displays a single cell of the map. */
    virtual void dispCell(TrackerCellType cellType, std::ostream &os);
    virtual void drawEnv(std::ostream &os) override;
    virtual void drawSimulationState(solver::BeliefNode const *belief,
            solver::State const &state,
            std::ostream &os) override;

    void drawEnvAndPos(std::ostream &os, GridPosition pos);


    /* ---------------------- Basic customizations  ---------------------- */
    virtual double getDefaultHeuristicValue(solver::HistoryEntry const *entry,
                solver::State const *state, solver::HistoricalData const *data) override;

    /* ------- Customization of more complex solver functionality  --------- */
    virtual std::vector<std::unique_ptr<solver::DiscretizedPoint>> getAllActionsInOrder();
    virtual std::unique_ptr<solver::ActionPool> createActionPool(solver::Solver *solver) override;

    virtual std::unique_ptr<solver::Serializer> createSerializer(solver::Solver *solver) override;

    /**
     * Returns proportion of belief particles about the target's
     * position for each grid position in the map
     */
    std::vector<std::vector<float>> getTargetPosBelief(solver::BeliefNode const *belief);

    
    /** Gets the expected coordinates after taking the given action;
     *  this may result in invalid coordinates.
     */
    GridPosition getNewPos(GridPosition const &position, int yaw, ActionType action);
    int getNewYaw(int yaw, ActionType action);

    /** Returns true iff the given GridPosition form a valid position. */
    bool isValid(GridPosition const &pos);

    TrackerOptions *options_;

    /** Returns true iff target is visible from robot */
    bool isTargetVisible(GridPosition const &robotPos, int robotYaw, GridPosition const &targetPos);

    /** Generates one new target state */
    TargetState getNextTargetState(GridPosition prevPos, int prevYaw);

  private:

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

    /** Returns distribution of possible target states next turn, 
     *  given the target's current position and yaw
     */
    std::unordered_set<TargetState, TargetStateHash> getNextTargetDist(GridPosition const &prevPos, int prevYaw); 
    /** Recursive function for getting target states for policy 0 */
    void getTargetStatesP0(std::unordered_set<TargetState, TargetStateHash> &states,
        GridPosition const &pos, int yaw, int numMoves);
    /** For zones policy mode. Return the current goal zone index */
    int getGoalZone(GridPosition const &pos);

    /** The penalty for each movement. */
    double moveCost_;
    /** The penalty for staying still (less tham moving) */
    double waitCost_;
    /** The penalty for being in front of human */
    double obstructCost_;
    /** The penalty for colliding with obstacle */
    double collideCost_;
    /** The reward for keeping the target in view. */
    double visibleReward_;

    /** The number of rows in the map. */
    long nRows_;
    /** The number of columns in the map. */
    long nCols_;

    /** The environment map in text form. */
    std::vector<std::string> mapText_;
    /** The environment map in vector form. */
    std::vector<std::vector<TrackerCellType>> envMap_;

    // General problem parameters
    long nActions_;

    /** The target policy. 0 for wall bouncing, 1 for zone waypoints */
    int targetPolicy_;

    /** Vars for policy 1 (zones mode) */
    std::vector<GridPosition> zones_;
    double targetP1MoveProbability_;
    
};
} /* namespace tracker */

#endif /* TRACKERMODEL_HPP_ */


