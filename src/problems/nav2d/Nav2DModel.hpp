#ifndef NAV2D_MODEL_HPP_
#define NAV2D_MODEL_HPP_

#include <ios>                          // for ostream
#include <memory>                       // for unique_ptr
#include <string>                       // for string
#include <utility>                      // for pair
#include <unordered_map>                // for unordered_map
#include <vector>                       // for vector

#include <boost/program_options.hpp>    // for variables_map

#include "problems/shared/geometry/Point2D.hpp"  // for Point2D
#include "problems/shared/geometry/Rectangle2D.hpp"  // for Rectangle2D
#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions

#include "solver/geometry/Action.hpp"            // for Action
#include "solver/geometry/Observation.hpp"       // for Observation
#include "solver/geometry/State.hpp"       // for State

#include "solver/mappings/enumerated_actions.hpp"
#include "solver/mappings/approximate_observations.hpp"

#include "solver/ChangeFlags.hpp"        // for ChangeFlags
#include "solver/Model.hpp"             // for Model::StepResult, Model

#include "global.hpp"                     // for RandomGenerator

namespace po = boost::program_options;

using geometry::Point2D;
using geometry::Rectangle2D;

namespace solver {
class ActionMapping;
class StatePool;
class EnumeratedPoint;
} /* namespace solver */

namespace nav2d {
class Nav2DAction;
class Nav2DState;
class Nav2DObservation;

class Nav2DModel : virtual public ModelWithProgramOptions,
    virtual public solver::ModelWithEnumeratedActions,
    virtual public solver::ModelWithApproximateObservations {

    friend class Nav2DTextSerializer;

  public:
    Nav2DModel(RandomGenerator *randGen, po::variables_map vm);
    ~Nav2DModel() = default;
    _NO_COPY_OR_MOVE(Nav2DModel);

    enum class ErrorType {
        PROPORTIONAL_GAUSSIAN_NOISE = 1,
        ABSOLUTE_GAUSSIAN_NOISE = 2
    };

    virtual std::string getName() override {
        return "Nav2D";
    }

    /***** Start implementation of Model's methods *****/
    // Simple getters
    virtual long getNStVars() override {
        return nStVars_;
    }
    virtual double getMinVal() override {
        return minVal_;
    }
    virtual double getMaxVal() override {
        return maxVal_;
    }

    // Other methods
    virtual std::unique_ptr<solver::State> sampleAnInitState() override;
    /** Generates a state uniformly at random. */
    virtual std::unique_ptr<solver::State> sampleStateUniform() override;

    virtual bool isTerminal(solver::State const &state) override;
    virtual double getHeuristicValue(solver::State const &state) override;
    virtual double getDefaultVal() override;

    /* --------------- Black box dynamics ----------------- */
    virtual std::unique_ptr<solver::State> generateNextState(
            solver::State const &state, solver::Action const &action) override;
    virtual std::unique_ptr<solver::Observation> generateObservation(
            solver::Action const &action,
            solver::State const &nextState) override;
    virtual double getReward(solver::State const &state,
                solver::Action const &action)  override;
    virtual Model::StepResult generateStep(solver::State const &state,
            solver::Action const &action) override;

    virtual std::vector<long> loadChanges(char const *changeFilename) override;
    virtual void update(long time, solver::StatePool *pool) override;

    enum class PointType {
        EMPTY = 0,
        START = 1,
        GOAL = 2,
        OBSERVATION = 3,
        OBSTACLE = 4,
        OUT_OF_BOUNDS = 5
    };
    /** Returns the contents of a given point on the map. */
    PointType getPointType(geometry::Point2D point);
    /** Displays an individual point on the map. */
    virtual void dispPoint(PointType type, std::ostream &os);
    virtual void drawEnv(std::ostream &os) override;
    virtual void drawState(solver::State const &state,
            std::ostream &os) override;

    virtual std::vector<std::unique_ptr<solver::EnumeratedPoint>>
    getAllActionsInOrder() override;
    virtual double getMaxObservationDistance() override;

  private:
    /** Parses an error type. */
    ErrorType parseErrorType(std::string text);
    /**
     * Finds and counts the rocks on the map, and initializes the required
     * data structures and variables.
     */
    void initialize();

    std::unique_ptr<Nav2DState> sampleStateAt(Point2D position);

    /**
     * Generates a next state for the given state and action;
     */
    std::unique_ptr<Nav2DState> makeNextState(
            Nav2DState const &state, Nav2DAction const &action);
    /** Generates an observation given a next state (i.e. after the action)
     * and an action.
     */
    std::unique_ptr<Nav2DObservation> makeObservation(
            Nav2DAction const &action,
            Nav2DState const &nextState);
    /** Retrieves the reward via the next state. */
    double makeReward(Nav2DState const &state,
            Nav2DAction const &action, Nav2DState const &nextState,
            bool isLegal);

    /** Amount of time per single time step. */
    double timeStepLength_;
    /** Penalty for crashing. */
    double crashPenalty_;
    /** Reward for reaching a goal area. */
    double goalReward_;

    /** Maximum speed allowed as an action. */
    double maxSpeed_;
    /** Cost per unit distance traveled. */
    double costPerUnitDistance_;
    /** Type of error used for the speed. */
    ErrorType speedErrorType_;
    /** Standard deviation for speed error. */
    double speedErrorSD_;

    /** Maximum rotational speed allowed in an action. */
    double maxRotationalSpeed_;
    /** Cost per revolution. */
    double costPerRevolution_;
    /** Type of error used for rotations. */
    ErrorType rotationErrorType_;
    /** Standard deviation for rotational error. */
    double rotationErrorSD_;

    typedef std::unordered_map<std::string, Rectangle2D> AreasByName;
    Rectangle2D mapArea_;
    AreasByName startAreas_;
    double totalStartArea_;
    AreasByName observationAreas_;
    AreasByName goalAreas_;
    AreasByName obstacles_;

    // Generic problem parameters
    long nStVars_;
    double minVal_, maxVal_;

    /** Maximum distance between observations to group them together. */
    double maxObservationDistance_;
};
} /* namespace nav2d */

#endif /* NAV2D_MODEL_HPP_ */
