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
#include "problems/shared/geometry/RTree.hpp"  // for RTree
#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions

#include "solver/geometry/Action.hpp"            // for Action
#include "solver/geometry/Observation.hpp"       // for Observation
#include "solver/geometry/State.hpp"       // for State

#include "solver/mappings/discretized_actions.hpp"
#include "solver/mappings/approximate_observations.hpp"

#include "solver/ChangeFlags.hpp"        // for ChangeFlags
#include "solver/Model.hpp"             // for Model::StepResult, Model

#include "Nav2DState.hpp"

#include "global.hpp"                     // for RandomGenerator

namespace po = boost::program_options;

namespace solver {
class ActionMapping;
class StatePool;
class EnumeratedPoint;
} /* namespace solver */

namespace nav2d {
class Nav2DAction;
class Nav2DObservation;

struct Nav2DTransition : public solver::TransitionParameters {
    double speed = 0.0;
    double rotationalSpeed = 0.0;
    double moveRatio = 0.0; // 0.0 = same position; 1.0 = full move
    bool reachedGoal = false;
    bool hadCollision = false;
};

class Nav2DModel : virtual public ModelWithProgramOptions,
    virtual public solver::ModelWithDiscretizedActions,
    virtual public solver::ModelWithApproximateObservations {

    friend class Nav2DAction;
    friend class Nav2DTextSerializer;
  public:
    Nav2DModel(RandomGenerator *randGen, po::variables_map vm);
    ~Nav2DModel() = default;
    _NO_COPY_OR_MOVE(Nav2DModel);

    typedef std::unordered_map<int64_t, geometry::Rectangle2D> AreasById;

    enum class ErrorType {
        PROPORTIONAL_GAUSSIAN_NOISE = 1,
        ABSOLUTE_GAUSSIAN_NOISE = 2,
        NONE = 3
    };

    enum class AreaType {
        EMPTY = 0,
        START = 1,
        GOAL = 2,
        OBSERVATION = 3,
        OBSTACLE = 4,
        OUT_OF_BOUNDS = 5,
        WORLD = 6
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
    virtual std::unique_ptr<solver::TransitionParameters> generateTransition(
                solver::State const &state,
                solver::Action const &action);

    virtual std::unique_ptr<solver::State> generateNextState(
            solver::State const &state,
            solver::Action const &action,
            solver::TransitionParameters const *tp) override;

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

    virtual Model::StepResult generateStep(
            solver::State const &state,
            solver::Action const &action) override;

    virtual std::vector<long> loadChanges(char const *changeFilename) override;
    virtual void update(long time, solver::StatePool *pool) override;

    /** Returns the tree with the given object type. */
    geometry::RTree *getTree(AreaType type);
    /** Returns the areas of the given type. */
    AreasById *getAreas(AreaType type);
    /** Returns the distance to the nearest object of the given type. */
    double getDistance(geometry::Point2D point, AreaType type);
    /** Returns true iff the point is inside an object of the given type. */
    bool isInside(geometry::Point2D point, AreaType type);
    /** Returns the contents of a given point on the map. */
    AreaType getAreaType(geometry::Point2D point);
    /** Displays an individual point on the map. */
    virtual void dispPoint(AreaType type, std::ostream &os);
    virtual void drawEnv(std::ostream &os) override;
    virtual void drawState(solver::State const &state,
            std::ostream &os) override;

    virtual long getNumberOfBins() override;
    virtual std::unique_ptr<solver::EnumeratedPoint> sampleAnAction(
            long code) override;

    virtual double getMaxObservationDistance() override;

  private:
    /** Converts an area type to a string. */
    std::string areaTypeToString(AreaType type);
    /** Parses an area type. */
    AreaType parseAreaType(std::string text);
    /** Parses an error type. */
    ErrorType parseErrorType(std::string text);
    /** Applies error to the speed. */
    double applySpeedError(double speed);
    /** Applies error to the rotational speed. */
    double applyRotationalError(double rotationalSpeed);

    /** Adds the given Rectangle2D as an area of the given type. */
    void addArea(int64_t id, geometry::Rectangle2D const &area,
            AreaType type);
    /** Samples a state at the given point. */
    std::unique_ptr<Nav2DState> sampleStateAt(geometry::Point2D position);

    /** Amount of time per single time step. */
    double timeStepLength_;
    /** Cost per unit time. */
    double costPerUnitTime_;
    /** Number of steps for interpolation. */
    long interpolationStepCount_;
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

    /** Maximum distance between observations to group them together. */
    double maxObservationDistance_;

    // Generic problem parameters
    long nStVars_;
    double minVal_, maxVal_;

    geometry::Rectangle2D mapArea_;
    AreasById startAreas_;
    double totalStartArea_;
    AreasById observationAreas_;
    AreasById goalAreas_;
    AreasById obstacles_;

    geometry::RTree obstacleTree_;
    geometry::RTree goalAreaTree_;
    geometry::RTree startAreaTree_;
    geometry::RTree observationAreaTree_;

    /** Represents a change in the Tag model. */
    struct Nav2DChange {
        std::string operation = "";
        AreaType type = AreaType::EMPTY;
        int64_t id = 0;
        geometry::Rectangle2D area{};
    };

    /** The changes (scheduled for simulation). */
    std::map<long, std::vector<Nav2DChange>> changes_;
};
} /* namespace nav2d */

#endif /* NAV2D_MODEL_HPP_ */
