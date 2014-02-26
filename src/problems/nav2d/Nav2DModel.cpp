#include "Nav2DModel.hpp"

#include <cmath>                        // for pow, floor
#include <cstddef>                      // for size_t
#include <cstdlib>                      // for exit

#include <fstream>                      // for operator<<, basic_ostream, endl, basic_ostream<>::__ostream_type, ifstream, basic_ostream::operator<<, basic_istream, basic_istream<>::__istream_type
#include <initializer_list>
#include <iostream>                     // for cout, cerr
#include <memory>                       // for unique_ptr, default_delete
#include <random>                       // for uniform_int_distribution, bernoulli_distribution
#include <set>                          // for set, _Rb_tree_const_iterator, set<>::iterator
#include <string>                       // for string, getline, char_traits, basic_string
#include <tuple>                        // for tie, tuple
#include <unordered_map>                // for unordered_map<>::value_type, unordered_map
#include <utility>                      // for move, pair, make_pair
#include <vector>                       // for vector, vector<>::reference, __alloc_traits<>::value_type, operator==

#include <boost/program_options.hpp>    // for variables_map, variable_value

#include "global.hpp"                     // for RandomGenerator, make_unique
#include "problems/shared/geometry/Point2D.hpp"
#include "problems/shared/geometry/Rectangle2D.hpp"

#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions

#include "solver/geometry/Action.hpp"            // for Action
#include "solver/geometry/Observation.hpp"       // for Observation
#include "solver/geometry/State.hpp"       // for State

#include "solver/mappings/enumerated_actions.hpp"
#include "solver/mappings/approximate_observations.hpp"

#include "solver/ChangeFlags.hpp"        // for ChangeFlags
#include "solver/Model.hpp"             // for Model::StepResult, Model
#include "solver/StatePool.hpp"

#include "Nav2DAction.hpp"         // for Nav2DAction
#include "Nav2DObservation.hpp"    // for Nav2DObservation
#include "Nav2DState.hpp"          // for Nav2DState

using std::cerr;
using std::cout;
using std::endl;

using geometry::Point2D;
using geometry::Rectangle2D;

namespace po = boost::program_options;

namespace nav2d {
Nav2DModel::Nav2DModel(RandomGenerator *randGen,
        po::variables_map vm) :
    ModelWithProgramOptions(randGen, vm),
    timeStepLength_(vm["problem.timeStepLength"].as<double>()),
    crashPenalty_(vm["problem.crashPenalty"].as<double>()),
    goalReward_(vm["problem.goalReward"].as<double>()),
    maxSpeed_(vm["problem.maxSpeed"].as<double>()),
    costPerUnitDistance_(vm["problem.costPerUnitDistance"].as<double>()),
    speedErrorType_(parseErrorType(
                vm["problem.speedErrorType"].as<std::string>())),
    speedErrorSD_(vm["problem.speedErrorSD"].as<double>()),
    maxRotationalSpeed_(vm["problem.maxRotationalSpeed"].as<double>()),
    costPerRevolution_(vm["problem.costPerRevolution"].as<double>()),
    rotationErrorType_(parseErrorType(
                vm["problem.rotationErrorType"].as<std::string>())),
    rotationErrorSD_(vm["problem.rotationErrorSD"].as<double>()),
    mapArea_(),
    startAreas_(),
    totalStartArea_(0),
    observationAreas_(),
    goalAreas_(),
    obstacles_(),
    nStVars_(2),
    minVal_(-(crashPenalty_ + maxSpeed_ * costPerUnitDistance_
            + maxRotationalSpeed_ * costPerRevolution_)
            / (1 - getDiscountFactor())),
    maxVal_(0),
    maxObservationDistance_(vm["SBT.maxObservationDistance"].as<double>())
         {
    // Read the map from the file.
    std::ifstream inFile;
    char const *mapPath = vm["problem.mapPath"].as<std::string>().c_str();
    inFile.open(mapPath);
    if (!inFile.is_open()) {
        std::cerr << "Failed to open " << mapPath << "\n";
        exit(1);
    }
    std::string line;
    while (std::getline(inFile, line)) {
        cerr << line << endl;
        std::istringstream iss(line);
        std::string type;
        std::string name;
        Rectangle2D rect;
        iss >> type >> name >> rect;
        if (type == "World") {
            mapArea_ = rect;
        } else if (type == "Start") {
            startAreas_.emplace(name, rect);
            totalStartArea_ += rect.getArea();
        } else if (type == "Observation") {
            observationAreas_.emplace(name, rect);
        } else if (type == "Goal") {
            observationAreas_.emplace(name, rect);
        }
    }
    inFile.close();

    initialize();
    cout << "Constructed the Nav2DModel" << endl;
    cout << "Discount: " << getDiscountFactor() << endl;
    cout << "nStVars: " << nStVars_ << endl;
    cout << "Random initial states:" << endl;
    cout << *sampleAnInitState() << endl;
    cout << *sampleAnInitState() << endl;
    cout << *sampleAnInitState() << endl;
    cout << *sampleAnInitState() << endl;

    cout << "nParticles: " << getNParticles() << endl;
    cout << "Random state drawn:" << endl;
    drawState(*sampleAnInitState(), cout);
}

Nav2DModel::ErrorType Nav2DModel::parseErrorType(std::string text) {
    if (text == "proportional gaussian noise") {
        return ErrorType::PROPORTIONAL_GAUSSIAN_NOISE;
    } else if (text == "absolute gaussian noise") {
        return ErrorType::ABSOLUTE_GAUSSIAN_NOISE;
    } else {
        cerr << "ERROR: Invalid error type - " << text;
        return ErrorType::PROPORTIONAL_GAUSSIAN_NOISE;
    }
}

void Nav2DModel::initialize() {
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
    for (AreasByName::value_type const &entry : startAreas_) {
        areaTotal += entry.second.getArea();
        if (areaValue < areaTotal) {
            return sampleStateAt(entry.second.sampleUniform(randGen));
        }
    }
    cerr << "ERROR: Invalid area at " << areaValue << endl;
    return nullptr;
}

std::unique_ptr<solver::State> Nav2DModel::sampleStateUniform() {
    return sampleStateAt(mapArea_.sampleUniform(*getRandomGenerator()));
}

bool Nav2DModel::isTerminal(solver::State const &state) {
    return getPointType(static_cast<Nav2DState const &>(
            state).getPosition()) == PointType::GOAL;
}

double Nav2DModel::getHeuristicValue(solver::State const &state) {
    Nav2DState const &rockSampleState =
        static_cast<Nav2DState const &>(state);
    // TODO Calculate distance to a goal square!
    return 0;
}

double Nav2DModel::getDefaultVal() {
    return minVal_;
}

std::unique_ptr<solver::State> Nav2DModel::generateNextState(
           solver::State const &state, solver::Action const &action) {
    // TODO Handle generation of next states;
    return nullptr;
}

std::unique_ptr<solver::Observation> Nav2DModel::generateObservation(
        solver::Action const &action, solver::State const &nextState) {
    Nav2DState const &navState = static_cast<Nav2DState const &>(nextState);
    if (getPointType(navState.getPosition()) == PointType::OBSERVATION) {
        return std::make_unique<Nav2DObservation>(navState);
    } else {
        return std::make_unique<Nav2DObservation>();
    }
}

double Nav2DModel::getReward(solver::State const &state,
        solver::Action const &action) {
    // TODO Handle rewards.
    Nav2DState const &rockSampleState =
        static_cast<Nav2DState const &>(state);
    return 0;
}

solver::Model::StepResult Nav2DModel::generateStep(
        solver::State const &state,
        solver::Action const &action) {
    Nav2DState const &rockSampleState =
        static_cast<Nav2DState const &>(state);
    solver::Model::StepResult result;
    result.action = action.copy();

    result.nextState = generateNextState(rockSampleState, action);
    result.observation = generateObservation(action, *result.nextState);
    result.immediateReward = getReward(rockSampleState, action);
    result.isTerminal = isTerminal(*result.nextState);
    return result;
}

std::vector<long> Nav2DModel::loadChanges(char const */*changeFilename*/) {
    std::vector<long> result;
    return result;
}

void Nav2DModel::update(long /*time*/, solver::StatePool */*pool*/) {
}


Nav2DModel::PointType Nav2DModel::getPointType(geometry::Point2D point) {
    if (!mapArea_.contains(point)) {
        return PointType::OUT_OF_BOUNDS;
    } else {
        return PointType::EMPTY;
    }
}

void Nav2DModel::dispPoint(Nav2DModel::PointType type, std::ostream &os) {
    switch(type) {
    case PointType::EMPTY:
        os << " ";
        return;
    case PointType::START:
        os << "+";
        return;
    case PointType::GOAL:
        os << "*";
        return;
    case PointType::OBSTACLE:
        os << "x";
        return;
    case PointType::OUT_OF_BOUNDS:
        os << "%";
        return;
    default:
        cerr << "ERROR: Invalid point type!?" << endl;
        return;
    }
}

void Nav2DModel::drawEnv(std::ostream &os) {
    double minX = mapArea_.getLowerLeft().getX();
    double maxX = mapArea_.getUpperRight().getX();
    double minY = mapArea_.getLowerLeft().getY();
    double maxY = mapArea_.getUpperRight().getY();
    double height = maxY - minY;
    long nRows = 30; //(int)height;
    double width = maxX - minX;
    long nCols = (int)width;
    for (long i = 0; i <= nRows + 1; i++) {
        double y = (nRows + 0.5 - i) * height / nRows;
        for (long j = 0; j <= nCols + 1; j++) {
            double x = (j - 0.5) * width / nCols;
            dispPoint(getPointType({x, y}), os);
        }
        os << endl;
    }
}

void Nav2DModel::drawState(solver::State const &state, std::ostream &os) {
    Nav2DState const &navState = static_cast<Nav2DState const &>(state);
    double minX = mapArea_.getLowerLeft().getX();
    double maxX = mapArea_.getUpperRight().getX();
    double minY = mapArea_.getLowerLeft().getY();
    double maxY = mapArea_.getUpperRight().getY();
    double height = maxY - minY;
    long nRows = 30; //(int)height;
    double width = maxX - minX;
    long nCols = (int)width;

    long stateI = nRows - (int)std::round(navState.getY() * nRows / height - 0.5);
    long stateJ = (int)std::round(navState.getX() * nCols / width + 0.5);
    for (long i = 0; i <= nRows + 1; i++) {
        double y = (nRows + 0.5 - i) * height / nRows;
        for (long j = 0; j <= nCols + 1; j++) {
            double x = (j - 0.5) * width / nCols;
            if (i == stateI && j == stateJ) {
                os << "o";
            } else {
                dispPoint(getPointType({x, y}), os);
            }
        }
        os << endl;
    }
    os << state << endl;
}

std::vector<std::unique_ptr<solver::EnumeratedPoint>>
Nav2DModel::getAllActionsInOrder() {
    std::vector<std::unique_ptr<solver::EnumeratedPoint>> allActions_;
    return allActions_;
}

double Nav2DModel::getMaxObservationDistance() {
    return maxObservationDistance_;
}
} /* namespace nav2d */
