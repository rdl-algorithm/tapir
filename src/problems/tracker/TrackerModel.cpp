#include "TrackerModel.hpp"

#include <cmath>                        // for floor, pow
#include <cstddef>                      // for size_t
#include <cstdlib>                      // for exit

#include <memory>
#include <fstream>                      // for ifstream, basic_istream, basic_istream<>::__istream_type
#include <iomanip>                      // for operator<<, setw
#include <iostream>                     // for cout
#include <random>                       // for uniform_int_distribution, bernoulli_distribution
#include <unordered_map>                // for _Node_iterator, operator!=, unordered_map<>::iterator, _Node_iterator_base, unordered_map
#include <utility>                      // for make_pair, move, pair

#include <boost/program_options.hpp>    // for variables_map, variable_value

#include "global.hpp"                     // for RandomGenerator, make_unique
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator!=, operator<<
#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions

#include "solver/abstract-problem/Action.hpp"            // for Action
#include "solver/abstract-problem/Model.hpp"             // for Model::StepResult, Model
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/abstract-problem/State.hpp"             // for State, State::Hash, operator<<, operator==

#include "solver/changes/ChangeFlags.hpp"        // for ChangeFlags

#include "solver/indexing/FlaggingVisitor.hpp"
#include "solver/indexing/RTree.hpp"
#include "solver/indexing/SpatialIndexVisitor.hpp"             // for State, State::Hash, operator<<, operator==

#include "solver/mappings/enumerated_actions.hpp"
#include "solver/mappings/discrete_observations.hpp"

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"
#include "solver/StatePool.hpp"

#include "TrackerAction.hpp"
#include "TrackerObservation.hpp"
#include "TrackerState.hpp"                 // for TrackerState

using std::cout;
using std::endl;
namespace po = boost::program_options;

namespace tracker {
TrackerModel::TrackerModel(RandomGenerator *randGen, po::variables_map vm, std::vector<std::vector<TrackerCellType>> envMap) :
    ModelWithProgramOptions(randGen, vm),
    moveCost_(vm["problem.moveCost"].as<double>()),
    trackerReward_(vm["problem.trackerReward"].as<double>()),
    failedTrackerPenalty_(vm["problem.failedTrackerPenalty"].as<double>()),
    targetStayProbability_(
            vm["problem.targetStayProbability"].as<double>()),
    nRows_(0), // to be updated
    nCols_(0), // to be updated
    mapText_(), // will be pushed to
    envMap_(), // will be pushed to
    changes_(),
    nActions_(4),
    nStVars_(5),
    minVal_(-failedTrackerPenalty_ / (1 - getDiscountFactor())),
    maxVal_(trackerReward_)
    {
    setAllActions(getAllActionsInOrder());

    
    if (envMap.size() > 0) {
        nRows_ = envMap.size();
        nCols_ = envMap[0].size();
        envMap_ = envMap;
    }
    else {
    
        // If environment map argument is empty, assume map is to be read from file
        std::ostringstream message;
        message << "No map supplied. Will read from file. ";
        std::ifstream inFile;
        char const *mapPath = vm["problem.mapPath"].as<std::string>().c_str();
        inFile.open(mapPath);
        if (!inFile.is_open()) {
            message << "ERROR: Failed to open " << mapPath;
            debug::show_message(message.str());
            std::exit(1);
        }
        inFile >> nRows_ >> nCols_;
        std::string tmp;
        getline(inFile, tmp);
        for (long i = 0; i < nRows_; i++) {
            getline(inFile, tmp);
            mapText_.push_back(tmp);
        }
        inFile.close();
        initialize();
    }

    if (hasVerboseOutput()) {
        cout << "Constructed the TrackerModel" << endl;
        cout << "Discount: " << getDiscountFactor() << endl;
        cout << "Size: " << nRows_ << " by " << nCols_ << endl;
        cout << "move cost: " << moveCost_ << endl;
        cout << "nActions: " << nActions_ << endl;
        cout << "nStVars: " << nStVars_ << endl;
        cout << "nParticles: " << getNParticles() << endl;
        cout << "Environment:" << endl << endl;
        //drawEnv(cout);
    }
}

void TrackerModel::initialize() {
    GridPosition p;
    envMap_.resize(nRows_);
    for (p.i = nRows_ - 1; p.i >= 0; p.i--) {
        envMap_[p.i].resize(nCols_);
        for (p.j = 0; p.j < nCols_; p.j++) {
            char c = mapText_[p.i][p.j];
            TrackerCellType cellType;
            if (c == 'X') {
                cellType = WALL;
            } else {
                cellType = TrackerCellType::EMPTY;
            }
            envMap_[p.i][p.j] = cellType;
        }
    }
}

GridPosition TrackerModel::randomEmptyCell() {
    GridPosition pos;
    while (true) {
        pos.i = std::uniform_int_distribution<long>(0, nRows_ - 1)(
                *getRandomGenerator());
        pos.j = std::uniform_int_distribution<long>(0, nCols_ - 1)(
                *getRandomGenerator());
        if (envMap_[pos.i][pos.j] == TrackerCellType::EMPTY) {
            break;
        }
    }
    return pos;
}

std::unique_ptr<solver::State> TrackerModel::sampleAnInitState() {
    return sampleStateUniform();
}

std::unique_ptr<solver::State> TrackerModel::sampleStateUniform() {

    // Random positions
    GridPosition robotPos = randomEmptyCell();
    GridPosition targetPos = randomEmptyCell();

    // Random yaw (discretised by 45 degrees)
    int i = rand() % 9 - 4; // Random int from -4 to 4
    int robotYaw = i * 45;
    i = rand() % 9 - 4;
    int targetYaw = i * 45;

    return std::make_unique<TrackerState>(robotPos, robotYaw,
        targetPos, targetYaw, false);
}

bool TrackerModel::isTerminal(solver::State const &state) {
    return static_cast<TrackerState const &>(state).isVisible();
}

double TrackerModel::getHeuristicValue(solver::State const &state) {
    if (!heuristicEnabled()) {
        return getDefaultVal();
    }

    TrackerState const &trackerState = static_cast<TrackerState const &>(state);
    if (trackerState.isVisible()) {
        return 0;
    }
    GridPosition robotPos = trackerState.getRobotPos();
    GridPosition targetPos = trackerState.getTargetPos();
    long dist = robotPos.manhattanDistanceTo(targetPos);
    double nSteps = dist / targetStayProbability_;
    double finalDiscount = std::pow(getDiscountFactor(), nSteps);
    double qVal = -moveCost_ * (1 - finalDiscount) / (1 - getDiscountFactor());
    qVal += finalDiscount * trackerReward_;
    return qVal;
}

std::pair<std::unique_ptr<TrackerState>, bool> TrackerModel::makeNextState(
        solver::State const &state, solver::Action const &action) {

    // Cast to Tracker types
    TrackerState const &trackerState = static_cast<TrackerState const &>(state);
    TrackerAction const &trackerAction = static_cast<TrackerAction const &>(action);

    // Get current positions and yaws
    GridPosition robotPos = trackerState.getRobotPos();
    GridPosition targetPos = trackerState.getTargetPos();
    int robotYaw = trackerState.getRobotYaw();
    int targetYaw = trackerState.getTargetYaw();

    // Find next state robot position and yaw
    GridPosition newRobotPos = getNewPos(robotPos, robotYaw, 
        trackerAction.getActionType());
    int newRobotYaw = getNewYaw(robotYaw, trackerAction.getActionType());

    // Sample target's action and get next target position and yaw
    std::vector<ActionType> targetActions(makeTargetActions());
    ActionType targetAction = targetActions[std::uniform_int_distribution<long>(
        0, targetActions.size() - 1)(*getRandomGenerator())];
    GridPosition newTargetPos = getNewPos(targetPos, targetYaw, targetAction);
    if (!isValid(newTargetPos))
        newTargetPos = targetPos;
    int newTargetYaw = getNewYaw(targetYaw, targetAction);

    if (!isValid(newRobotPos)) {
        return std::make_pair(
                std::make_unique<TrackerState>(robotPos, robotYaw,
                    newTargetPos, newTargetYaw, false),
                false);
    }
    return std::make_pair(std::make_unique<TrackerState>(
                    newRobotPos, newTargetYaw, newTargetPos,
                    newTargetYaw, false), true);
}

std::vector<ActionType> TrackerModel::makeTargetActions() {
    std::vector<ActionType> actions;
    actions.push_back(ActionType::FORWARD);
    actions.push_back(ActionType::FORWARD);
    actions.push_back(ActionType::FORWARD);
    actions.push_back(ActionType::TURN_RIGHT);
    actions.push_back(ActionType::TURN_LEFT);
    return actions;
}

GridPosition TrackerModel::getNewPos(GridPosition const &oldPos, int yaw,
        ActionType action) {
    GridPosition newPos = oldPos;

    switch (action) {
    case ActionType::FORWARD:

        // Precondition: -180 <= yaw <= 180
        if (abs(yaw) < 90)
            newPos.j += 1;
        else if (abs(yaw) > 90)
            newPos.j -= 1;
        if (yaw > 0 && yaw < 180)
            newPos.i -= 1;
        else if (yaw < 0 && yaw > -180)
            newPos.i += 1;
        break;
    case ActionType::TURN_RIGHT:
        break;
    case ActionType::TURN_LEFT:
        break;
    case ActionType::REVERSE:
        if (abs(yaw) < 90)
            newPos.j -= 1;
        else if (abs(yaw) > 90)
            newPos.j += 1;
        if (yaw > 0 && yaw < 180)
            newPos.i += 1;
        else if (yaw < 0 && yaw > -180)
            newPos.i -= 1;
        break;
    case ActionType::WAIT:
        break;
    default:
        std::ostringstream message;
        message << "Invalid action: " << (long)action;
        debug::show_message(message.str());
        break;
    }

    return newPos;
}

int TrackerModel::getNewYaw(int yaw, ActionType action) {
    if (action == ActionType::TURN_RIGHT)
        return yaw - 45;
    else if (action == ActionType::TURN_LEFT)
        return yaw + 45;
    else
        return yaw;
}

bool TrackerModel::isValid(GridPosition const &position) {
    return (position.i >= 0 && position.i < nRows_ && position.j >= 0
            && position.j < nCols_ && envMap_[position.i][position.j] != WALL);
}

std::unique_ptr<solver::Observation> TrackerModel::makeObservation(
        solver::Action const & /*action*/,
        TrackerState const &nextState) {
    return std::make_unique<TrackerObservation>(nextState.getRobotPos(),
             nextState.getRobotPos().manhattanDistanceTo(nextState.getTargetPos()) <= 1);
}

double TrackerModel::generateReward(    // TODO
        solver::State const &state,
        solver::Action const &action,
        solver::TransitionParameters const */*tp*/,
        solver::State const */*nextState*/) {

    double reward;
    TrackerState tState = static_cast<TrackerState const &>(state);

    long dist = tState.getRobotPos().manhattanDistanceTo(
        tState.getTargetPos());
    if (dist > 0 && dist <= 2) {
        reward = 5;
    }


    return reward -moveCost_;
}

std::unique_ptr<solver::State> TrackerModel::generateNextState(
        solver::State const &state,
        solver::Action const &action,
        solver::TransitionParameters const */*tp*/) {
    return makeNextState(static_cast<TrackerState const &>(state), action).first;
}

std::unique_ptr<solver::Observation> TrackerModel::generateObservation(
        solver::State const */*state*/,
        solver::Action const &action,
        solver::TransitionParameters const */*tp*/,
        solver::State const &nextState) {
    return makeObservation(action, static_cast<TrackerState const &>(nextState));
}

solver::Model::StepResult TrackerModel::generateStep(solver::State const &state,
        solver::Action const &action) {
    solver::Model::StepResult result;
    result.action = action.copy();
    std::unique_ptr<TrackerState> nextState = makeNextState(state, action).first;

    result.observation = makeObservation(action, *nextState);
    result.reward = generateReward(state, action, nullptr, nullptr);
    result.isTerminal = isTerminal(*nextState);
    result.nextState = std::move(nextState);
    return result;
}

std::vector<std::unique_ptr<solver::State>> TrackerModel::generateParticles(
        solver::BeliefNode */*previousBelief*/,
        solver::Action const &action, solver::Observation const &obs,
        std::vector<solver::State const *> const &previousParticles) {
    std::vector<std::unique_ptr<solver::State>> newParticles;
    TrackerObservation const &observation = (
            static_cast<TrackerObservation const &>(obs));
    ActionType actionType = (
            static_cast<TrackerAction const &>(action).getActionType());

    struct Hash {
        std::size_t operator()(TrackerState const &state) const {
            return state.hash();
        }
    };
    typedef std::unordered_map<TrackerState, double, Hash> WeightMap;
    WeightMap weights;
    double weightTotal = 0;

    GridPosition newRobotPos(observation.getRobotPos());
    int newRobotYaw = observation.getRobotYaw();
    cout << "JOSH: " << newRobotPos.i << " " << newRobotPos.j << " " << newRobotYaw << endl;
    for (solver::State const *state : previousParticles) {
        TrackerState const *trackerState = static_cast<TrackerState const *>(state);
        GridPosition oldRobotPos(trackerState->getRobotPos());
        int oldRobotYaw = trackerState->getRobotYaw();
        // Ignore states that do not match knowledge of the robot's position/yaw.
        if (newRobotPos != getNewPos(oldRobotPos, oldRobotYaw, actionType) ||
            newRobotYaw != getNewYaw(oldRobotYaw, actionType)) {
            continue;
        }
        GridPosition oldTargetPos(trackerState->getTargetPos());
        int oldTargetYaw(trackerState->getTargetYaw());
        std::vector<ActionType> actions(makeTargetActions());
        std::vector<ActionType> newActions;
        for (ActionType targetAction : actions) {
            if (getNewPos(oldTargetPos, oldTargetYaw,
                    targetAction) != newRobotPos) {
                newActions.push_back(targetAction);
            }
        }
        double probability = 1.0 / newActions.size();
        for (ActionType targetAction : newActions) {
            GridPosition newTargetPos = getNewPos(oldTargetPos,
                oldTargetYaw, targetAction);
            int newTargetYaw = getNewYaw(oldTargetYaw, targetAction);
            TrackerState newState(newRobotPos, newRobotYaw,
                newTargetPos, newTargetYaw, false);
            weights[newState] += probability;
            weightTotal += probability;
        }
    }
    double scale = getNParticles() / weightTotal;
    for (WeightMap::iterator it = weights.begin(); it != weights.end();
         it++) {
        double proportion = it->second * scale;
        long numToAdd = static_cast<long>(proportion);
        if (std::bernoulli_distribution(proportion-numToAdd)(
                *getRandomGenerator())) {
            numToAdd += 1;
        }
        for (int i = 0; i < numToAdd; i++) {
            newParticles.push_back(std::make_unique<TrackerState>(it->first));
        }
    }
    
    return newParticles;
}

std::vector<std::unique_ptr<solver::State>> TrackerModel::generateParticles(
        solver::BeliefNode */*previousBelief*/,
        solver::Action const &action, solver::Observation const &obs) {
    std::vector<std::unique_ptr<solver::State>> newParticles;
    TrackerObservation const &observation = (
            static_cast<TrackerObservation const &>(obs));
    ActionType actionType = (
            static_cast<TrackerAction const &>(action).getActionType());
    GridPosition newRobotPos(observation.getRobotPos());

    while (newParticles.size() < getNParticles()) {
        std::unique_ptr<solver::State> state = sampleStateUniform();
        solver::Model::StepResult result = generateStep(*state, action);
        if (obs == *result.observation) {
            newParticles.push_back(std::move(result.nextState));
        }
    }

    return newParticles;
}

std::vector<long> getVector(std::istringstream &iss) {
    std::vector<long> values;
    std::string tmpStr;
    std::getline(iss, tmpStr, '(');
    std::getline(iss, tmpStr, ')');
    std::istringstream sstr(tmpStr);
    while (std::getline(sstr, tmpStr, ',')) {
        values.push_back(std::atoi(tmpStr.c_str()));
    }
    return values;
}

std::vector<long> TrackerModel::loadChanges(char const *changeFilename) {
    std::vector<long> changeTimes;
    std::ifstream ifs;
    ifs.open(changeFilename);
    std::string line;
    while (std::getline(ifs, line)) {
        std::string tmpStr;
        long time;
        long nChanges;
        std::istringstream(line) >> tmpStr >> time >> tmpStr >> nChanges;

        changes_[time] = std::vector<TrackerChange>();
        changeTimes.push_back(time);
        for (int i = 0; i < nChanges; i++) {
            std::getline(ifs, line);
            TrackerChange change;
            std::istringstream sstr(line);
            std::getline(sstr, change.changeType, ':');
            std::vector<long> v0 = getVector(sstr);
            std::vector<long> v1 = getVector(sstr);
            change.i0 = v0[0];
            change.j0 = v0[1];
            change.i1 = v1[0];
            change.j1 = v1[1];
            changes_[time].push_back(change);
        }
    }
    ifs.close();
    return changeTimes;
}

void TrackerModel::update(long time, solver::StatePool *pool) {
    for (TrackerChange &change : changes_[time]) {
        if (hasVerboseOutput()) {
            cout << change.changeType << " " << change.i0 << " " << change.j0;
            cout << " " << change.i1 << " " << change.j1 << endl;
        }
        if (change.changeType == "Add Obstacles") {
            for (long i = static_cast<long>(change.i0); i <= change.i1; i++) {
                for (long j = static_cast<long>(change.j0); j <= change.j1; j++) {
                    envMap_[i][j] = TrackerCellType::WALL;
                }
            }
            solver::RTree *tree =
                    static_cast<solver::RTree *>(pool->getStateIndex());
            solver::FlaggingVisitor visitor(pool,
                    solver::ChangeFlags::DELETED);
            tree->boxQuery(visitor,
                    std::vector<double> { change.i0, change.j0, 0,       0,               0 },
                    std::vector<double> { change.i1, change.j1, nRows_-1.0,  nCols_-1.0,  1 });
            tree->boxQuery(visitor,
                    std::vector<double> { 0,           0,           change.i0, change.j0, 0 },
                    std::vector<double> { nRows_-1.0,  nCols_-1.0,  change.i1, change.j1, 1 });
        } else if (change.changeType == "Remove Obstacles") {
            for (long i = static_cast<long>(change.i0); i <= change.i1; i++) {
                for (long j = static_cast<long>(change.j0); j <= change.j1; j++) {
                    envMap_[i][j] = TrackerCellType::EMPTY;
                }
            }
            solver::RTree *tree =
                    static_cast<solver::RTree *>(pool->getStateIndex());
            solver::FlaggingVisitor visitor(pool, solver::ChangeFlags::TRANSITION);
            tree->boxQuery(visitor,
                    std::vector<double> { change.i0-1, change.j0-1, 0,             0,             0 },
                    std::vector<double> { change.i1+1, change.j1+1, nRows_-1.0,    nCols_-1.0,    1 });
            tree->boxQuery(visitor,
                    std::vector<double> { 0,             0,             change.i0-1, change.j0-1, 0 },
                    std::vector<double> { nRows_-1.0,    nCols_-1.0,    change.i1+1, change.j1+1, 1 });
        }
    }
}

void TrackerModel::dispCell(TrackerCellType cellType, std::ostream &os) {
    if (cellType >= EMPTY) {
        os << std::setw(2);
        os << cellType;
        return;
    }
    switch (cellType) {
    case WALL:
        os << "XX";
        break;
    default:
        os << "ERROR-" << cellType;
        break;
    }
}

void TrackerModel::drawEnv(std::ostream &os) {
    for (std::vector<TrackerCellType> &row : envMap_) {
        for (TrackerCellType cellType : row) {
            dispCell(cellType, os);
            os << " ";
        }
        os << endl;
    }
}

void TrackerModel::drawSimulationState(solver::BeliefNode *belief,
        solver::State const &state,
        std::ostream &os) {
    TrackerState const &trackerState = static_cast<TrackerState const &>(state);
    std::vector<solver::State const *> particles = belief->getStates();
    std::vector<std::vector<long>> particleCounts(nRows_,
            std::vector<long>(nCols_));
    for (solver::State const *particle : particles) {
        GridPosition targetPos = static_cast<TrackerState const &>(
                *particle).getTargetPos();
        particleCounts[targetPos.i][targetPos.j] += 1;
    }

    std::vector<int> colors {196,
        161, 126, 91, 56, 21,
        26, 31, 36, 41, 46};
    if (hasColorOutput()) {
        os << "Color map: ";
        for (int color : colors) {
            os << "\033[38;5;" << color << "m";
            os << '*';
            os << "\033[0m";
        }
        os << endl;
    }
    for (std::size_t i = 0; i < envMap_.size(); i++) {
        for (std::size_t j = 0; j < envMap_[0].size(); j++) {
            double proportion = (double) particleCounts[i][j]
                    / particles.size();
            if (hasColorOutput()) {
                if (proportion > 0) {
                    int color = colors[proportion * (colors.size() - 1)];
                    os << "\033[38;5;" << color << "m";
                }
            }
            GridPosition pos(i, j);
            bool hasRobot = (pos == trackerState.getRobotPos());
            bool hasTarget = (pos == trackerState.getTargetPos());
            if (hasRobot) {
                if (hasTarget) {
                    os << "#";
                } else {
                    os << "r";
                }
            } else if (hasTarget) {
                os << "o";
            } else {
                if (envMap_[i][j] == WALL) {
                    os << "X";
                } else {
                    os << ".";
                }
            }
            if (hasColorOutput()) {
                os << "\033[0m";
            }
        }
        os << endl;
    }
}

std::vector<std::unique_ptr<solver::DiscretizedPoint>>
TrackerModel::getAllActionsInOrder() {
    std::vector<std::unique_ptr<solver::DiscretizedPoint>> allActions;
    for (long code = 0; code < nActions_; code++) {
        allActions.push_back(std::make_unique<TrackerAction>(code));
    }
    return allActions;
}
} /* namespace tracker */
