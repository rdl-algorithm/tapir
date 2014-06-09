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

#include "solver/abstract-problem/Action.hpp"            // for Action
#include "solver/abstract-problem/Model.hpp"             // for Model::StepResult, Model
#include "solver/abstract-problem/ModelChange.hpp"            // for ModelChange
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/abstract-problem/State.hpp"       // for State

#include "solver/mappings/actions/discretized_actions.hpp"
#include "solver/mappings/observations/approximate_observations.hpp"

#include "solver/changes/ChangeFlags.hpp"        // for ChangeFlags


#include "Nav2DState.hpp"

#include "global.hpp"                     // for RandomGenerator

namespace po = boost::program_options;

namespace solver {
class ActionMapping;
class StatePool;
class DiscretizedPoint;
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
    bool hitBounds = false;

    void print(std::ostream &os) const override;
};

class Nav2DModel : virtual public ModelWithProgramOptions {

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

    /* ----------------------- Basic getters  ------------------- */
    virtual std::string getName() override {
        return "Nav2D";
    }


    /* ---------- Virtual getters for ABT / model parameters  ---------- */
    // Simple getters
    virtual long getNumberOfStateVariables() override {
        return nStVars_;
    }
    virtual double getMinVal() override {
        return minVal_;
    }
    virtual double getMaxVal() override {
        return maxVal_;
    }


    /* --------------- The model interface proper ----------------- */
    virtual std::unique_ptr<solver::State> sampleAnInitState() override;
    /** Generates a state uniformly at random. */
    virtual std::unique_ptr<solver::State> sampleStateUniform() override;
    virtual bool isTerminal(solver::State const &state) override;


    /* -------------------- Black box dynamics ---------------------- */
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


    /* -------------- Methods for handling model changes ---------------- */
    virtual void applyChange(solver::ModelChange const &change, solver::StatePool *pool) override;

    /** Returns the tree with the given object type. */
    geometry::RTree *getTree(AreaType type);
    /** Returns the areas of the given type. */
    AreasById *getAreas(AreaType type);
    /** Returns the nearest point of the given type. */
    geometry::Point2D getClosestPointOfType(geometry::Point2D point,
            AreaType type);
    /** Returns the distance to the nearest object of the given type. */
    double getDistance(geometry::Point2D point, AreaType type);
    /** Returns true iff the point is inside an object of the given type. */
    bool isInside(geometry::Point2D point, AreaType type);
    /** Returns the contents of a given point on the map. */
    AreaType getAreaType(geometry::Point2D point);


    /* ------------------- Pretty printing methods --------------------- */
    /** Displays an individual point on the map. */
    virtual void dispPoint(AreaType type, std::ostream &os);
    virtual void drawEnv(std::ostream &os) override;
    virtual void drawSimulationState(solver::BeliefNode const *belief,
            solver::State const &state,
            std::ostream &os) override;

    /* ---------------------- Basic customizations  ---------------------- */
    virtual double getHeuristicValue(solver::HistoricalData const *data,
            solver::State const *state);


    /* ------- Customization of more complex solver functionality  --------- */
    virtual std::unique_ptr<solver::ActionPool> createActionPool(solver::Solver *solver)
                override;
    virtual std::unique_ptr<solver::ObservationPool> createObservationPool(solver::Solver *solver)
            override;
    virtual std::unique_ptr<solver::Serializer> createSerializer(solver::Solver *solver) override;

  private:
    /** Extrapolates the position, based on a current position, speed,
     * rotational speed, and the proportion of the time step spent moving.
     */
    std::unique_ptr<Nav2DState> extrapolateState(Nav2DState const &state,
            double speed, double rotationalSpeed, double moveRatio = 1.0);
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
    /** Penalty for hitting the bounds. */
    double boundsHitPenalty_;
    /** Reward for reaching a goal area. */
    double goalReward_;

    /** Maximum speed allowed as an action. */
    double maxSpeed_;
    /** Cost per unit distance traveled. */
    double costPerUnitDistance_;
    /** Standard deviation for speed error. */
    double speedErrorSD_;
    /** Type of error used for the speed. */
    ErrorType speedErrorType_;

    /** Maximum rotational speed allowed in an action. */
    double maxRotationalSpeed_;
    /** Cost per revolution. */
    double costPerRevolution_;
    /** Standard deviation for rotational error. */
    double rotationErrorSD_;
    /** Type of error used for rotations. */
    ErrorType rotationErrorType_;

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

  public:
    double getTimeStepLength() const { return timeStepLength_; }
    double getMaxSpeed() const { return maxSpeed_; }
    double getCostPerUnitDistance() const { return costPerUnitDistance_; }
    double getMaxRotationalSpeed() const { return maxRotationalSpeed_; }
    double getCostPerRevolution() const { return costPerRevolution_; }
};


/** Represents a change in the Tag model. */
struct Nav2DChange : public solver::ModelChange {
    std::string operation = "";
    Nav2DModel::AreaType type = Nav2DModel::AreaType::EMPTY;
    int64_t id = 0;
    geometry::Rectangle2D area{};
};

class Nav2DActionsPool : public solver::DiscretizedActionPool {

};
} /* namespace nav2d */

#endif /* NAV2D_MODEL_HPP_ */
