#ifndef TAGMODEL_HPP_
#define TAGMODEL_HPP_

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

#include "TagAction.hpp"
#include "TagOptions.hpp"
#include "TagMdpSolver.hpp"

namespace solver {
class StatePool;
} /* namespace solver */

namespace tag {
class TagObervation;
class TagState;

/** Represents a change in the Tag model. */
struct TagChange : public solver::ModelChange {
    std::string changeType = "";
    long i0 = 0;
    long i1 = 0;
    long j0 = 0;
    long j1 = 0;
};

class TagUBParser : public shared::Parser<solver::HeuristicFunction> {
public:
    TagUBParser(TagModel *model);
    virtual ~TagUBParser() = default;
    _NO_COPY_OR_MOVE(TagUBParser);

    virtual solver::HeuristicFunction parse(solver::Solver *solver, std::vector<std::string> args);
private:
    TagModel *model_;
};

class TagModel: public shared::ModelWithProgramOptions {
    friend class TagObservation;
    friend class TagMdpSolver;

  public:
    TagModel(RandomGenerator *randGen, std::unique_ptr<TagOptions> options);
    ~TagModel() = default;
    TagModel(TagModel const &) = delete;
    TagModel(TagModel &&) = delete;
    TagModel &operator=(TagModel const &) = delete;
    TagModel &operator=(TagModel &&) = delete;

    /** The cells are either empty or walls. */
    enum class TagCellType : int {
        EMPTY = 0,
        WALL = -1
    };

    /******************** Added by Josh **************************/

    /** Get 2D vector representing the current environment map */
    inline const std::vector<std::vector<TagCellType>>& getEnvMap() {
        return envMap_;
    }

    /**
     * Returns proportion of belief particles about the target's
     * position for each grid position in the map
     */
    std::vector<std::vector<float>> getBeliefProportions(solver::BeliefNode const *belief);

    /* ---------- Custom getters for extra functionality  ---------- */
    long getNRows() const {
        return nRows_;
    }
    long getNCols() const {
        return nCols_;
    }

    /** Sets up the MDP solver for this model. */
    void makeMdpSolver() {
        mdpSolver_ = std::make_unique<TagMdpSolver>(this);
        mdpSolver_->solve();
    }
    /** Returns the MDP solver for this model. */
    TagMdpSolver *getMdpSolver() {
        return mdpSolver_.get();
    }

    /* --------------- The model interface proper ----------------- */
    // Other virtual methods
    std::unique_ptr<solver::State> sampleAnInitState() override;
    /** Generates an untagged state uniformly at random. */
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
    virtual void dispCell(TagCellType cellType, std::ostream &os);
    virtual void drawEnv(std::ostream &os) override;
    virtual void drawSimulationState(solver::BeliefNode const *belief,
            solver::State const &state,
            std::ostream &os) override;


    /* ---------------------- Basic customizations  ---------------------- */
    virtual double getDefaultHeuristicValue(solver::HistoryEntry const *entry,
                solver::State const *state, solver::HistoricalData const *data) override;

    virtual double getUpperBoundHeuristicValue(solver::State const &state);

    /* ------- Customization of more complex solver functionality  --------- */
    virtual std::vector<std::unique_ptr<solver::DiscretizedPoint>> getAllActionsInOrder();
    virtual std::unique_ptr<solver::ActionPool> createActionPool(solver::Solver *solver) override;

    virtual std::unique_ptr<solver::Serializer> createSerializer(solver::Solver *solver) override;

  private:
    /** Initialises the required data structures and variables */
    void initialize();

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
    std::unique_ptr<solver::Observation> makeObservation(
            solver::Action const &action, TagState const &nextState);


    /** Generates the distribution for the opponent's actions. */
    std::vector<ActionType> makeOpponentActions(GridPosition const &robotPos,
            GridPosition const &opponentPos);
    /** Generates a proper distribution for next opponent positions. */
    std::unordered_map<GridPosition, double> getNextOpponentPositionDistribution(
            GridPosition const &robotPos, GridPosition const &opponentPos);

    /** Moves the opponent. */
    GridPosition sampleNextOpponentPosition(GridPosition const &robotPos,
            GridPosition const &opponentPos);

    /** Gets the expected coordinates after taking the given action;
     *  this may result in invalid coordinates.
     */
    std::pair<GridPosition, bool> getMovedPos(GridPosition const &position, ActionType action);
    /** Returns true iff the given GridPosition form a valid position. */
    bool isValid(GridPosition const &pos);

    TagOptions *options_;

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

    /** The environment map in text form. */
    std::vector<std::string> mapText_;
    /** The environment map in vector form. */
    std::vector<std::vector<TagCellType>> envMap_;

    // General problem parameters
    long nActions_;

    /** Solver for the MDP version of the problem. */
    std::unique_ptr<TagMdpSolver> mdpSolver_;
};
} /* namespace tag */

#endif /* TAGMODEL_HPP_ */
