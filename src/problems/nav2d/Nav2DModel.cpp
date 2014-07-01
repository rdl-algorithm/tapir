#include "Nav2DModel.hpp"

#define _USE_MATH_DEFINES
#include <cmath>                        // for pow, floor
#include <cstddef>                      // for size_t
#include <cstdlib>                      // for exit

#include <fstream>                      // for operator<<, basic_ostream, endl, basic_ostream<>::__ostream_type, ifstream, basic_ostream::operator<<, basic_istream, basic_istream<>::__istream_type
#include <initializer_list>
#include <iostream>                     // for cout
#include <memory>                       // for unique_ptr, default_delete
#include <random>                       // for uniform_int_distribution, bernoulli_distribution
#include <set>                          // for set, _Rb_tree_const_iterator, set<>::iterator
#include <string>                       // for string, getline, char_traits, basic_string
#include <tuple>                        // for tie, tuple
#include <unordered_map>                // for unordered_map<>::value_type, unordered_map
#include <utility>                      // for move, pair, make_pair
#include <vector>                       // for vector, vector<>::reference, __alloc_traits<>::value_type, operator==

#include "global.hpp"                     // for RandomGenerator, make_unique
#include "problems/shared/geometry/Point2D.hpp"
#include "problems/shared/geometry/Vector2D.hpp"
#include "problems/shared/geometry/Rectangle2D.hpp"
#include "problems/shared/geometry/utilities.hpp"

#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions

#include "solver/abstract-problem/Action.hpp"            // for Action
#include "solver/abstract-problem/Model.hpp"             // for Model::StepResult, Model
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/abstract-problem/State.hpp"       // for State

#include "solver/changes/ChangeFlags.hpp"        // for ChangeFlags

#include "solver/mappings/actions/discretized_actions.hpp"
#include "solver/mappings/observations/approximate_observations.hpp"

#include "solver/indexing/RTree.hpp"
#include "solver/indexing/FlaggingVisitor.hpp"

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"
#include "solver/StatePool.hpp"

#include "Nav2DAction.hpp"         // for Nav2DAction
#include "Nav2DActionPool.hpp"         // for Nav2DActionPool
#include "Nav2DObservation.hpp"    // for Nav2DObservation
#include "Nav2DOptions.hpp"
#include "Nav2DState.hpp"          // for Nav2DState
#include "Nav2DTextSerializer.hpp"

using std::cout;
using std::endl;

using geometry::Point2D;
using geometry::Vector2D;
using geometry::Rectangle2D;
using geometry::RTree;

namespace nav2d {
void Nav2DTransition::print(std::ostream &os) const {
    os << speed << "/" << rotationalSpeed << " ";
    os << moveRatio << " ";
    if (reachedGoal) {
        os << "G";
    }
    if (hadCollision) {
        os << "C";
    }
    if (hitBounds) {
        os << "B";
    }
}

Nav2DModel::Nav2DModel(RandomGenerator *randGen, std::unique_ptr<Nav2DOptions> options) :
    shared::ModelWithProgramOptions("Nav2D", randGen, std::move(options)),
    options_(const_cast<Nav2DOptions *>(static_cast<Nav2DOptions const *>(getOptions()))),
    nDimensions_(2),
    timeStepLength_(options_->timeStepLength),
    costPerUnitTime_(options_->costPerUnitTime),
    interpolationStepCount_(options_->interpolationStepCount),
    crashPenalty_(options_->crashPenalty),
    boundsHitPenalty_(options_->boundsHitPenalty),
    goalReward_(options_->goalReward),
    maxSpeed_(options_->maxSpeed),
    costPerUnitDistance_(options_->costPerUnitDistance),
    speedErrorSD_(options_->speedErrorSD),
    speedErrorType_(speedErrorSD_ <= 0 ? ErrorType::NONE : parseErrorType(
            options_->speedErrorType)),
    maxRotationalSpeed_(options_->maxRotationalSpeed),
    costPerRevolution_(options_->costPerRevolution),
    rotationErrorSD_(options_->rotationErrorSD),
    rotationErrorType_(rotationErrorSD_ <= 0 ? ErrorType::NONE : parseErrorType(
            options_->rotationErrorType)),
    maxObservationDistance_(options_->maxObservationDistance),
    mapArea_(),
    startAreas_(),
    totalStartArea_(0),
    observationAreas_(),
    goalAreas_(),
    obstacles_(),
    obstacleTree_(nDimensions_),
    goalAreaTree_(nDimensions_),
    startAreaTree_(nDimensions_),
    observationAreaTree_(nDimensions_) {

    options_->numberOfStateVariables = nDimensions_;
    options_->minVal = -(crashPenalty_ + maxSpeed_ * costPerUnitDistance_
            + maxRotationalSpeed_ * costPerRevolution_) / (1 - options_->discountFactor);
    options_->maxVal = goalReward_;

    // Read the map from the file.
    std::ifstream inFile;
    inFile.open(options_->mapPath);
    if (!inFile.is_open()) {
        std::ostringstream message;
        message << "Failed to open " << options_->mapPath;
        debug::show_message(message.str());
        std::exit(1);
    }
    std::string line;
    while (std::getline(inFile, line)) {
        std::string typeString;
        int64_t id;
        Rectangle2D rect;
        std::istringstream(line) >> typeString >> id >> rect;
        AreaType areaType = parseAreaType(typeString);
        if (areaType == AreaType::WORLD) {
            mapArea_ = rect;
        } else {
            addArea(id, rect, areaType);
        }
    }
    inFile.close();

    if (options_->hasVerboseOutput) {
        cout << "Constructed the Nav2DModel" << endl;
        cout << "Discount: " << options_->discountFactor << endl;
        cout << "historiesPerStep: " << options_->historiesPerStep << endl;
        cout << endl << endl;
    }
}

std::string Nav2DModel::areaTypeToString(Nav2DModel::AreaType type) {
    switch(type) {
    case AreaType::EMPTY:
        return "Empty";
    case AreaType::WORLD:
        return "World";
    case AreaType::START:
        return "Start";
    case AreaType::OBSERVATION:
        return "Observation";
    case AreaType::GOAL:
        return "Goal";
    case AreaType::OBSTACLE:
        return "Obstacle";
    case AreaType::OUT_OF_BOUNDS:
        return "OOB";
    default:
        std::ostringstream message;
        message << "ERROR: Invalid area code: " << static_cast<long>(type);
        debug::show_message(message.str());
        return "ERROR";
     }
}

Nav2DModel::AreaType Nav2DModel::parseAreaType(std::string text) {
    if (text == "World") {
        return AreaType::WORLD;
    } else if (text == "Start") {
        return AreaType::START;
    } else if (text == "Observation") {
        return AreaType::OBSERVATION;
    } else if (text == "Goal") {
        return AreaType::GOAL;
    } else if (text == "Obstacle") {
        return AreaType::OBSTACLE;
    } else if (text == "Empty") {
        return AreaType::EMPTY;
    } else if (text == "OOB") {
        return AreaType::OUT_OF_BOUNDS;
    } else {
        std::ostringstream message;
        message << "ERROR: Invalid area type: " << text;
        debug::show_message(message.str());
        return AreaType::EMPTY;
    }
}

Nav2DModel::ErrorType Nav2DModel::parseErrorType(std::string text) {
    if (text == "proportional gaussian noise") {
        return ErrorType::PROPORTIONAL_GAUSSIAN_NOISE;
    } else if (text == "absolute gaussian noise") {
        return ErrorType::ABSOLUTE_GAUSSIAN_NOISE;
    } else if (text == "none") {
        return ErrorType::NONE;
    } else {
        std::ostringstream message;
        message << "ERROR: Invalid error type - " << text;
        debug::show_message(message.str());
        return ErrorType::PROPORTIONAL_GAUSSIAN_NOISE;
    }
}

double Nav2DModel::applySpeedError(double speed) {
    switch(speedErrorType_) {
    case ErrorType::PROPORTIONAL_GAUSSIAN_NOISE:
        speed = std::normal_distribution<double>(1.0, speedErrorSD_)(
                *getRandomGenerator()) * speed;
        if (speed < 0) {
            speed = 0;
        }
        return speed;
    case ErrorType::ABSOLUTE_GAUSSIAN_NOISE:
        speed = std::normal_distribution<double>(speed, speedErrorSD_)(
                *getRandomGenerator());
        if (speed < 0) {
            speed = 0;
        }
        return speed;
    case ErrorType::NONE:
        return speed;
    default:
        debug::show_message("Cannot calculate speed error");
        return speed;
    }
}

double Nav2DModel::applyRotationalError(double rotationalSpeed) {
    switch(rotationErrorType_) {
    case ErrorType::PROPORTIONAL_GAUSSIAN_NOISE:
        return rotationalSpeed * std::normal_distribution<double>(
                1.0, rotationErrorSD_)(*getRandomGenerator());
    case ErrorType::ABSOLUTE_GAUSSIAN_NOISE:
        return std::normal_distribution<double>(
                rotationalSpeed, rotationErrorSD_)(*getRandomGenerator());
    case ErrorType::NONE:
        return rotationalSpeed;
    default:
        debug::show_message("Cannot calculate rotational error");
        return rotationalSpeed;
    }
}

void Nav2DModel::addArea(int64_t id, Rectangle2D const &area,
        Nav2DModel::AreaType type) {
    getAreas(type)->emplace(id, area);
    std::vector<double> lowCorner = area.getLowerLeft().asVector();
    std::vector<double> highCorner = area.getUpperRight().asVector();
    SpatialIndex::Region region(&lowCorner[0], &highCorner[0], nDimensions_);
    getTree(type)->getTree()->insertData(0, nullptr, region, id);
    if (type == AreaType::START) {
        totalStartArea_ += area.getArea();
    }
}

std::unique_ptr<Nav2DState> Nav2DModel::sampleStateAt(Point2D position) {
    return std::make_unique<Nav2DState>(position,
            -std::uniform_real_distribution<double>(-0.5, 0.5)(
                    *getRandomGenerator()),
                    costPerUnitDistance_, costPerRevolution_);
}

std::unique_ptr<solver::State> Nav2DModel::sampleAnInitState() {
    RandomGenerator &randGen = *getRandomGenerator();
    double areaValue = std::uniform_real_distribution<double>(0,
            totalStartArea_)(randGen);
    double areaTotal = 0;
    for (AreasById::value_type const &entry : startAreas_) {
        areaTotal += entry.second.getArea();
        if (areaValue < areaTotal) {
            return std::make_unique<Nav2DState>(
                    entry.second.sampleUniform(randGen), 0,
                    costPerUnitDistance_, costPerRevolution_);
        }
    }
    std::ostringstream message;
    message << "ERROR: Invalid area at " << areaValue;
    debug::show_message(message.str());
    return nullptr;
}

std::unique_ptr<solver::State> Nav2DModel::sampleStateUniform() {
    return sampleStateAt(mapArea_.sampleUniform(*getRandomGenerator()));
}

bool Nav2DModel::isTerminal(solver::State const &state) {
    return isInside(static_cast<Nav2DState const &>(state).getPosition(),
            AreaType::GOAL);
}

std::unique_ptr<Nav2DState> Nav2DModel::extrapolateState(
        Nav2DState const &state, double speed,
        double rotationalSpeed, double moveRatio) {
    Point2D position = state.getPosition();
    double direction = state.getDirection();
    Point2D newPosition;
    double newDirection;
    if (rotationalSpeed == 0.0) {
        newDirection = direction;
        newPosition = position + Vector2D(moveRatio * speed * timeStepLength_,
                direction);
    } else {
        newDirection = direction + moveRatio * rotationalSpeed * timeStepLength_;
        newPosition = position;
        if (speed != 0.0) {
            double absSpeed = std::abs(rotationalSpeed);
            double radius = speed / (2 * M_PI * absSpeed);
            double absDeltaInRadians = 2 * M_PI * moveRatio * absSpeed * timeStepLength_;
            double forwardDistance = radius * std::sin(absDeltaInRadians);
            double sideDistance = radius - (radius * std::cos(absDeltaInRadians));
            newPosition += Vector2D(forwardDistance, direction);
            double sideDirection = direction + (rotationalSpeed > 0 ? 0.25 : -0.25);
            newPosition += Vector2D(sideDistance, sideDirection);
        }
    }
    return std::make_unique<Nav2DState>(newPosition, newDirection,
            costPerUnitDistance_,
            costPerRevolution_);
}

std::unique_ptr<solver::TransitionParameters> Nav2DModel::generateTransition(
               solver::State const &state,
               solver::Action const &action) {
    Nav2DState const &navState = static_cast<Nav2DState const &>(state);
    Nav2DAction const &navAction = static_cast<Nav2DAction const &>(action);
    std::unique_ptr<Nav2DTransition> transition(
            std::make_unique<Nav2DTransition>());

    transition->speed = applySpeedError(navAction.getSpeed());
    transition->rotationalSpeed = applyRotationalError(
            navAction.getRotationalSpeed());
    for (long step = 1; step <= interpolationStepCount_; step++) {
        double previousRatio = transition->moveRatio;
        transition->moveRatio = (double)step / interpolationStepCount_;

        std::unique_ptr<Nav2DState> currentState = extrapolateState(
                navState, transition->speed, transition->rotationalSpeed,
                transition->moveRatio);
        Point2D currentPosition = currentState->getPosition();
        if (!mapArea_.contains(currentPosition)) {
            transition->moveRatio = previousRatio;
            transition->hitBounds = true;
            break;
         }
        if (isInside(currentPosition, AreaType::OBSTACLE)) {
            transition->moveRatio = previousRatio;
            transition->hadCollision = true;
            break;
        }
        if (isInside(currentPosition, AreaType::GOAL)){
            transition->reachedGoal = true;
            break;
        }
    }
    return std::move(transition);

}

std::unique_ptr<solver::State> Nav2DModel::generateNextState(
        solver::State const &state,
        solver::Action const &/*action*/,
        solver::TransitionParameters const *tp) {
    Nav2DTransition const &tp2 = static_cast<Nav2DTransition const &>(*tp);
    Nav2DState const &navState = static_cast<Nav2DState const &>(state);
    return extrapolateState(navState, tp2.speed, tp2.rotationalSpeed,
            tp2.moveRatio);
}


std::unique_ptr<solver::Observation> Nav2DModel::generateObservation(
        solver::State const */*state*/,
        solver::Action const &/*action*/,
        solver::TransitionParameters const */*tp*/,
        solver::State const &nextState) {
    Nav2DState const &navState = static_cast<Nav2DState const &>(nextState);
    if (isInside(navState.getPosition(), AreaType::OBSERVATION)) {
        // debug::show_message("NOTE: Generated an observation!", true, false);
        return std::make_unique<Nav2DObservation>(navState);
    } else {
        return std::make_unique<Nav2DObservation>();
    }
}

double Nav2DModel::generateReward(
        solver::State const &/*state*/,
        solver::Action const &/*action*/,
        solver::TransitionParameters const *tp,
        solver::State const */*nextState*/) {
    Nav2DTransition const &tp2 = static_cast<Nav2DTransition const &>(*tp);
    double reward = 0;
    reward -= costPerUnitTime_ * timeStepLength_;
    double distance = std::abs(tp2.moveRatio * tp2.speed * timeStepLength_);
    double turnAmount = std::abs(tp2.moveRatio * tp2.rotationalSpeed
            * timeStepLength_);
    reward -= costPerUnitDistance_ * distance;
    reward -= costPerRevolution_ * turnAmount;
    if (tp2.reachedGoal) {
        reward += goalReward_;
    }
    if (tp2.hadCollision) {
        reward -= crashPenalty_;
    }
    if (tp2.hitBounds) {
        reward -= boundsHitPenalty_;
    }
    return reward;
}

solver::Model::StepResult Nav2DModel::generateStep(
        solver::State const &state,
        solver::Action const &action) {
    solver::Model::StepResult result;
    result.action = action.copy();
    result.transitionParameters = generateTransition(state, action);
    result.nextState = generateNextState(state, action,
            result.transitionParameters.get());
    result.observation = generateObservation(nullptr, action,
            nullptr, *result.nextState);
    result.reward = generateReward(state, action,
            result.transitionParameters.get(), result.nextState.get());
    result.isTerminal = static_cast<Nav2DTransition const &>(
            *result.transitionParameters).reachedGoal;
    return result;
}

void Nav2DModel::applyChanges(std::vector<std::unique_ptr<solver::ModelChange>> const &changes,
        solver::Solver *solver) {
    solver::StatePool *pool = nullptr;
    if (solver != nullptr) {
        pool = solver->getStatePool();
    }

    for (auto const &change : changes) {
        Nav2DChange const &navChange = static_cast<Nav2DChange const &>(*change);
        addArea(navChange.id, navChange.area, navChange.type);
        if (pool == nullptr) {
            continue;
        }

        // Obstacles => delete states.
        solver::FlaggingVisitor visitor(pool, solver::ChangeFlags::DELETED);
        solver::RTree *tree = static_cast<solver::RTree *>(pool->getStateIndex());

        // Observation areas => update observation on the previous step.
        if (navChange.type == AreaType::OBSERVATION) {
            visitor.flagsToSet_ = solver::ChangeFlags::OBSERVATION_BEFORE;
        }
        tree->boxQuery(visitor,
                {navChange.area.getLowerLeft().getX(),
                        navChange.area.getLowerLeft().getY(), -2.0},
                {navChange.area.getUpperRight().getX(),
                        navChange.area.getUpperRight().getY(), +2.0});
    }
}

geometry::RTree *Nav2DModel::getTree(AreaType type) {
    switch(type) {
    case AreaType::GOAL:
        return &goalAreaTree_;
    case AreaType::OBSTACLE:
        return &obstacleTree_;
    case AreaType::START:
        return &startAreaTree_;
    case AreaType::OBSERVATION:
        return &observationAreaTree_;
    default:
        std::ostringstream message;
        message << "ERROR: Cannot get tree; type " << static_cast<long>(type);
        debug::show_message(message.str());
        return nullptr;
    }
}

Nav2DModel::AreasById *Nav2DModel::getAreas(AreaType type) {
    switch(type) {
    case AreaType::GOAL:
        return &goalAreas_;
    case AreaType::OBSTACLE:
        return &obstacles_;
    case AreaType::START:
        return &startAreas_;
    case AreaType::OBSERVATION:
        return &observationAreas_;
    default:
        std::ostringstream message;
        message << "ERROR: Cannot get area; type " << static_cast<long>(type);
        debug::show_message(message.str());
        return nullptr;
    }
}

bool Nav2DModel::isInside(geometry::Point2D point, AreaType type) {
    for (AreasById::value_type &entry : *getAreas(type)) {
        if (entry.second.contains(point)) {
            return true;
        }
    }
    return false;
    /*
    geometry::RTree *tree = getTree(type);
    SpatialIndex::Point p(&(point.asVector()[0]), nStVars_);
    class MyVisitor: public SpatialIndex::IVisitor {
    public:
        bool isInside = false;
        void visitNode(SpatialIndex::INode const &) {}
        void visitData(std::vector<SpatialIndex::IData const *> &) {}
        void visitData(SpatialIndex::IData const &) {
            isInside = true;
        }
    };
    MyVisitor v;
    tree->getTree()->pointLocationQuery(p, v);
    return v.isInside;
    */
}

Point2D Nav2DModel::getClosestPointOfType(Point2D point, AreaType type) {
    double infinity = std::numeric_limits<double>::infinity();
    double distance = infinity;
    Point2D closestPoint(infinity, infinity);
    for (AreasById::value_type &entry : *getAreas(type)) {
        Point2D newClosestPoint = entry.second.closestPointTo(point);
        double newDistance = (point - newClosestPoint).getMagnitude();
        if (newDistance < distance) {
            distance = newDistance;
            closestPoint = newClosestPoint;
        }
    }
    return closestPoint;
}

double Nav2DModel::getDistance(Point2D point, AreaType type) {
    double distance = std::numeric_limits<double>::infinity();
    for (AreasById::value_type &entry : *getAreas(type)) {
        double newDistance = entry.second.distanceTo(point);
        if (newDistance < distance) {
            distance = newDistance;
        }
    }
    return distance;
    /*
    geometry::RTree *tree = getTree(type);
    SpatialIndex::Point p(&(point.asVector()[0]), nStVars_);
    class MyVisitor: public SpatialIndex::IVisitor {
    public:
        SpatialIndex::Point &p_;
        double distance_;
        MyVisitor(SpatialIndex::Point &p) :
                p_(p), distance_(std::numeric_limits<double>::infinity()) {}
        void visitNode(SpatialIndex::INode const &) {}
        void visitData(std::vector<SpatialIndex::IData const *> &) {
        }
        void visitData(SpatialIndex::IData const &data) {
            SpatialIndex::IShape *shape;
            data.getShape(&shape);
            distance_ = shape->getMinimumDistance(p_);
        }
    };
    MyVisitor v(p);
    tree->getTree()->nearestNeighborQuery(1, p, v);
    return v.distance_;
    */
}

Nav2DModel::AreaType Nav2DModel::getAreaType(geometry::Point2D point) {
    if (!mapArea_.contains(point)) {
        return AreaType::OUT_OF_BOUNDS;
    } else if (isInside(point, AreaType::OBSTACLE)) {
        return AreaType::OBSTACLE;
    } else if (isInside(point, AreaType::GOAL)) {
        return AreaType::GOAL;
    } else if (isInside(point, AreaType::START)) {
        return AreaType::START;
    } else if (isInside(point, AreaType::OBSERVATION)) {
        return AreaType::OBSERVATION;
    } else {
        return AreaType::EMPTY;
    }
}

void Nav2DModel::dispPoint(Nav2DModel::AreaType type, std::ostream &os) {
    switch(type) {
    case AreaType::EMPTY:
        os << " ";
        return;
    case AreaType::START:
        os << "+";
        return;
    case AreaType::GOAL:
        os << "*";
        return;
    case AreaType::OBSTACLE:
        os << "%";
        return;
    case AreaType::OBSERVATION:
        os << "x";
        return;
    case AreaType::OUT_OF_BOUNDS:
        os << "#";
        return;
    default:
        std::ostringstream message;
        message << "ERROR: Invalid point type!?";
        debug::show_message(message.str());
        return;
    }
}

void Nav2DModel::drawEnv(std::ostream &os) {
    double minX = mapArea_.getLowerLeft().getX();
    double maxX = mapArea_.getUpperRight().getX();
    double minY = mapArea_.getLowerLeft().getY();
    double maxY = mapArea_.getUpperRight().getY();
    double height = maxY - minY;
    long nRows = (long)height / 2;
    double width = maxX - minX;
    long nCols = (long)width;
    for (long i = 0; i <= nRows + 1; i++) {
        double y = (nRows + 0.5 - i) * height / nRows;
        for (long j = 0; j <= nCols + 1; j++) {
            double x = (j - 0.5) * width / nCols;
            dispPoint(getAreaType({x, y}), os);
        }
        os << endl;
    }
}

void Nav2DModel::drawSimulationState(solver::BeliefNode const *belief,
        solver::State const &state,
        std::ostream &os) {
    Nav2DState const &navState = static_cast<Nav2DState const &>(state);
    std::vector<solver::State const *> particles = belief->getStates();
    double minX = mapArea_.getLowerLeft().getX();
    double maxX = mapArea_.getUpperRight().getX();
    double minY = mapArea_.getLowerLeft().getY();
    double maxY = mapArea_.getUpperRight().getY();
    double height = maxY - minY;
    long nRows = (long)height / 2;
    double width = maxX - minX;
    long nCols = (long)width;

    os << "Belief has " << particles.size() << " particles." << endl;
    std::vector<std::vector<long>> particleCounts (nRows + 2,
            std::vector<long>(nCols + 2));

    for (solver::State const *particle : particles) {
        Nav2DState const &navParticle = static_cast<Nav2DState const &>(
                *particle);
        long pI = nRows - (int)std::round(
                navParticle.getY() * nRows / height - 0.5);
        long pJ = (int)std::round(navParticle.getX() * nCols / width + 0.5);
        particleCounts[pI][pJ] += 1;
    }


    std::vector<int> colors {22, 28, 34, 40, 46,
            82, 118, 154, 190, 226,
            227, 228, 229, 230, 231 };
    if (options_->hasColorOutput) {
        os << "Color map: ";
        for (int color : colors) {
            os << "\033[38;5;" << color << "m";
            os << '*';
            os << "\033[0m";
        }
        os << endl;
    }
    long stateI = nRows - (int)std::round(navState.getY() * nRows / height - 0.5);
    long stateJ = (int)std::round(navState.getX() * nCols / width + 0.5);
    double proportion = (double)particleCounts[stateI][stateJ] / particles.size();
    os << "Ratio in same square: "  << proportion << endl;
    for (long i = 0; i <= nRows + 1; i++) {
        double y = (nRows + 0.5 - i) * height / nRows;
        for (long j = 0; j <= nCols + 1; j++) {
            double x = (j - 0.5) * width / nCols;
            proportion = (double)particleCounts[i][j] / particles.size();
            bool isState = (i == stateI && j == stateJ);
            if (proportion <= 0 && !isState) {
                dispPoint(getAreaType( { x, y }), os);
                continue;
            }
            if (options_->hasColorOutput) {
                int color = colors[proportion * (colors.size() - 1)];
                os << "\033[38;5;" << color << "m";
            }
            if (isState) {
                if (proportion == 0) {
                    os << "!";
                } else {
                    os << (char) ('A' + proportion * 25);
                }
            } else {
                os << (char) ('a' + proportion * 25);
            }
            if (options_->hasColorOutput) {
                os << "\033[0m";
            }
        }
        os << endl;
    }
}


double Nav2DModel::getDefaultHeuristicValue(solver::HistoryEntry const */*entry*/,
        solver::State const *state, solver::HistoricalData const */*data*/) {
    Nav2DState const &nav2DState = static_cast<Nav2DState const &>(*state);
    Point2D closestPoint = getClosestPointOfType(nav2DState.getPosition(),
            AreaType::GOAL);
    Vector2D displacement = closestPoint - nav2DState.getPosition();
    double distance = displacement.getMagnitude();
    double turnAmount = std::abs(geometry::normalizeTurn(
            displacement.getDirection() - nav2DState.getDirection()));
    long numSteps = std::floor(distance / (maxSpeed_ * timeStepLength_));
    numSteps += std::floor(turnAmount / (maxRotationalSpeed_
            * timeStepLength_));
    if (numSteps <= 0) {
        numSteps = 1;
    }
    double costPerStep = costPerUnitDistance_ * distance / numSteps;
    costPerStep += costPerRevolution_ * turnAmount / numSteps;
    costPerStep += costPerUnitTime_ * timeStepLength_;

    double discountFactor = options_->discountFactor;
    double reward = 0;
    if (discountFactor < 1.0) {
        double finalDiscount = std::pow(discountFactor, numSteps);
        reward = finalDiscount * goalReward_;
        reward -= costPerStep * (1 - finalDiscount) / (1 - discountFactor);
    } else {
        reward = goalReward_ - costPerStep * numSteps;
    }
    if (!std::isfinite(reward)) {
        debug::show_message("Bad reward!");
    }
    return reward;
}


std::unique_ptr<solver::ActionPool> Nav2DModel::createActionPool(solver::Solver */*solver*/) {
    return std::make_unique<Nav2DActionPool>(this);
}
std::unique_ptr<solver::ObservationPool> Nav2DModel::createObservationPool(
        solver::Solver */*solver*/) {
    return std::make_unique<solver::ApproximateObservationPool>(maxObservationDistance_);
}
std::unique_ptr<solver::Serializer> Nav2DModel::createSerializer(solver::Solver *solver) {
    return std::make_unique<Nav2DTextSerializer>(solver);
}
} /* namespace nav2d */
