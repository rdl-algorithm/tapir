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
#include "TrackerState.hpp"

using std::cout;
using std::endl;
namespace po = boost::program_options;

namespace tracker {

TrackerModel::TrackerModel(RandomGenerator *randGen, po::variables_map vm) :
	ModelWithProgramOptions(randGen, vm),
	moveCost_(vm["problem.moveCost"].as<double>()),
    obstructCost_(vm["problem.obstructCost"].as<double>()),
	visibleReward_(vm["problem.visibleReward"].as<double>()),
	nRows_(0), // to be updated
	nCols_(0), // to be updated
    mapText_(), // will be pushed to
    envMap_(),  // will be pushed to
    nActions_(5),
    nStVars_(5),
    minVal_(0),
    maxVal_(visibleReward_)
{
	setAllActions(getAllActionsInOrder());

    if (hasVerboseOutput()) {
        cout << "Constructed the TrackerModel" << endl;
        cout << "Discount: " << getDiscountFactor() << endl;
        cout << "move cost: " << moveCost_ << endl;
        cout << "obstruct  cost: " << obstructCost_ << endl;
        cout << "visible reward: " << visibleReward_ << endl;
        cout << "nActions: " << nActions_ << endl;
        cout << "nStVars: " << nStVars_ << endl;
        cout << "minParticleCount: " << getMinParticleCount() << endl;
        cout << "histories per step: " << getNumberOfHistoriesPerStep() << endl;
        cout << "max depth: " << getMaximumDepth() << endl;
        cout << "Environment:" << endl << endl;
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

void TrackerModel::setEnvMap(std::vector<std::vector<TrackerCellType>> envMap) {
	if (envMap.size() < 1 || envMap[0].size() < 1) {
		cout << "Empty map" << endl;
		return;
	}
	nRows_ = envMap.size();
	nCols_ = envMap.size();
	envMap_ = envMap;

    if (hasVerboseOutput()) {
        cout << "Got environment map" << endl;
        cout << "Rows: " << nRows_ << endl;
        cout << "Cols: " << nCols_ << endl;
    }
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

    bool seesTarget = isTargetVisible(robotPos, robotYaw, targetPos);

    return std::make_unique<TrackerState>(robotPos, robotYaw,
        targetPos, targetYaw, seesTarget);
}

bool TrackerModel::isTerminal(solver::State const &state) {
    return false;
}

double TrackerModel::getHeuristicValue(solver::State const &state) {
    return 0;

    /*if (!heuristicEnabled()) {
        return getDefaultVal();
    }
    TrackerState const &TrackerState = static_cast<TrackerState const &>(state);
    GridPosition robotPos = trackerState.getRobotPosition();
    GridPosition targetPos = trackerState.getTargetPosition();
    long dist = robotPos.manhattanDistanceTo(targetPos);
    double nSteps = dist;// / targetStayProbability_;
    double finalDiscount = std::pow(getDiscountFactor(), nSteps);
    double qVal = -moveCost_ * (1 - finalDiscount) / (1 - getDiscountFactor());
    qVal += finalDiscount * visibleReward_;
    return qVal;*/
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

    // Sample target's action and get next position and yaw
    std::vector<ActionType> targetActions(makeTargetActions());
    ActionType targetAction = targetActions[std::uniform_int_distribution<long>(
        0, targetActions.size() - 1)(*getRandomGenerator())];
    GridPosition newTargetPos = getNewPos(targetPos, targetYaw, targetAction);
    if (!isValid(newTargetPos))
        newTargetPos = targetPos;
    int newTargetYaw = getNewYaw(targetYaw, targetAction);

    if (!isValid(newRobotPos)) {
        bool seesTarget = isTargetVisible(robotPos, robotYaw, newTargetPos);
        return std::make_pair(
                std::make_unique<TrackerState>(robotPos, robotYaw,
                    newTargetPos, newTargetYaw, seesTarget),
                false);
    }
    bool seesTarget = isTargetVisible(newRobotPos, newRobotYaw, newTargetPos);
    return std::make_pair(std::make_unique<TrackerState>(
                    newRobotPos, newRobotYaw, newTargetPos,
                    newTargetYaw, seesTarget), true);
}

std::vector<ActionType> TrackerModel::makeTargetActions() {
    std::vector<ActionType> actions;
    //actions.push_back(ActionType::FORWARD);
    //actions.push_back(ActionType::TURN_RIGHT);
    //actions.push_back(ActionType::TURN_LEFT);
    actions.push_back(ActionType::WAIT);
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
        yaw -= 45;
    else if (action == ActionType::TURN_LEFT)
        yaw += 45;

    // Ensure -180 <= yaw <= 180
    while (yaw > 180)
        yaw -= 360;
    while (yaw < -180)
        yaw += 360;
    return yaw;
}

bool TrackerModel::isValid(GridPosition const &position) {
    return (position.i >= 0 && position.i < nRows_ && position.j >= 0
            && position.j < nCols_ && envMap_[position.i][position.j] != WALL);
}

bool TrackerModel::isTargetVisible(GridPosition const &robotPos, int robotYaw,
    GridPosition const &targetPos) {

    GridPosition forwardPos = getNewPos(robotPos, robotYaw, ActionType::FORWARD);
    GridPosition forwardPos2 = getNewPos(forwardPos, robotYaw, ActionType::FORWARD);
    if (targetPos == forwardPos || targetPos == forwardPos2)
        return true;
    return false;
}

std::unique_ptr<solver::Observation> TrackerModel::makeObservation(
        solver::Action const & /*action*/, TrackerState const &nextState) {
    bool seesTarget = isTargetVisible(nextState.getRobotPos(),
        nextState.getRobotYaw(), nextState.getTargetPos());
    return std::make_unique<TrackerObservation>(nextState.getRobotPos(),
    	nextState.getRobotYaw(), seesTarget);
}

double TrackerModel::generateReward(
        solver::State const &state,
        solver::Action const &action,
        solver::TransitionParameters const */*tp*/,
        solver::State const */*nextState*/) {

    double reward = 0;
    TrackerState tState = static_cast<TrackerState const &>(state);
    GridPosition robotPos(tState.getRobotPos());
    GridPosition targetPos(tState.getTargetPos());
    int targetYaw = tState.getTargetYaw();
    if (robotPos == targetPos || 
        robotPos == getNewPos(targetPos, targetYaw, ActionType::FORWARD))
        reward -= obstructCost_;
    if (tState.seesTarget())
        reward += visibleReward_;
    reward -= moveCost_;
    return reward;
}

std::unique_ptr<solver::State> TrackerModel::generateNextState(
        solver::State const &state,
        solver::Action const &action,
        solver::TransitionParameters const */*tp*/) {
    return makeNextState(state, action).first;
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
    result.isTerminal = isTerminal(state);
    result.nextState = std::move(nextState);
    return result;
}


std::vector<std::unique_ptr<solver::State>> TrackerModel::generateParticles(
        solver::BeliefNode */*previousBelief*/, solver::Action const &action,
        solver::Observation const &obs,
        long nParticles,
        std::vector<solver::State const *> const &previousParticles) {
    std::vector<std::unique_ptr<solver::State>> newParticles;
    TrackerObservation const &observation = (static_cast<TrackerObservation const &>(obs));
    ActionType actionType = (static_cast<TrackerAction const &>(action).getActionType());

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
    /*
    if (observation.seesTarget()) {
        // If we saw the target, we must be in the same place.
        newParticles.push_back(
                std::make_unique<TrackerState>(newRobotPos, newRobotPos,
                        actionType == ActionType::TAG));
    } else {*/
        // We didn't see the target, so we must be in different places.
        for (solver::State const *state : previousParticles) {
            TrackerState const *trackerState = static_cast<TrackerState const *>(state);
            GridPosition oldRobotPos(trackerState->getRobotPos());
            int oldRobotYaw = trackerState->getRobotYaw();
            // Ignore states that do not match knowledge of the robot's position.
            if (newRobotPos != getNewPos(oldRobotPos, oldRobotYaw, actionType) ||
            	newRobotYaw != getNewYaw(oldRobotYaw, actionType)) {
                continue;
            }
            GridPosition oldTargetPos(trackerState->getTargetPos());
            int oldTargetYaw = trackerState->getTargetYaw();
            std::vector<ActionType> actions(makeTargetActions());
            std::vector<ActionType> newActions;
            for (ActionType targetAction : actions) {
                if (getNewPos(oldTargetPos, oldTargetYaw, targetAction) != newRobotPos) {
                    newActions.push_back(targetAction);
                }
            }
            double probability = 1.0 / newActions.size();
            for (ActionType targetAction : newActions) {
                GridPosition newTargetPos = getNewPos(oldTargetPos, oldTargetYaw, targetAction);
                int newTargetYaw = getNewYaw(oldTargetYaw, targetAction);
                bool seesTarget = isTargetVisible(newRobotPos, newRobotYaw, newTargetPos);
                TrackerState newState(newRobotPos, newRobotYaw, newTargetPos, newTargetYaw, seesTarget);
                weights[newState] += probability;
                weightTotal += probability;
            }
        }
        double scale = nParticles / weightTotal;
        for (WeightMap::iterator it = weights.begin(); it != weights.end();
                it++) {
            double proportion = it->second * scale;
            long numToAdd = static_cast<long>(proportion);
            if (std::bernoulli_distribution(proportion - numToAdd)(
                    *getRandomGenerator())) {
                numToAdd += 1;
            }
            for (int i = 0; i < numToAdd; i++) {
                newParticles.push_back(std::make_unique<TrackerState>(it->first));
            }
        }
    //}
    return newParticles;
}


std::vector<std::unique_ptr<solver::State>> TrackerModel::generateParticles(
        solver::BeliefNode */*previousBelief*/, solver::Action const &action,
        solver::Observation const &obs, long nParticles) {
    std::vector<std::unique_ptr<solver::State>> newParticles;
    TrackerObservation const &observation = (static_cast<TrackerObservation const &>(obs));
    ActionType actionType = (static_cast<TrackerAction const &>(action).getActionType());
    GridPosition newRobotPos(observation.getRobotPos());
    /*if (observation.seesTarget()) {
        // If we saw the target, we must be in the same place.
        while ((long)newParticles.size() < nParticles) {
            newParticles.push_back(
                std::make_unique<TrackerState>(newRobotPos, newRobotPos,
                        actionType == ActionType::TRACKER));
        }
    } else {*/
        while ((long)newParticles.size() < nParticles) {
            std::unique_ptr<solver::State> state = sampleStateUniform();
            solver::Model::StepResult result = generateStep(*state, action);
            if (obs == *result.observation) {
                newParticles.push_back(std::move(result.nextState));
            }
        }
    //}
    return newParticles;
}

void TrackerModel::applyChange(solver::ModelChange const &change,
        solver::StatePool *pool) {
}

void TrackerModel::dispCell(TrackerCellType cellType, std::ostream &os) {
}

void TrackerModel::drawEnv(std::ostream &os) {
}

void TrackerModel::drawSimulationState(solver::BeliefNode const *belief,
        solver::State const &state, std::ostream &os) {
}

std::vector<std::unique_ptr<solver::DiscretizedPoint>> TrackerModel::getAllActionsInOrder() {
    std::vector<std::unique_ptr<solver::DiscretizedPoint>> allActions;
    for (long code = 0; code < nActions_; code++) {
        allActions.push_back(std::make_unique<TrackerAction>(code));
    }
    return allActions;
}

} /* namespace tracker */