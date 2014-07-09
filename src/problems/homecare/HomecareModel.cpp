/** @file HomecareModel.cpp
 *
 * Contains the implementations for the core functionality of the Homecare POMDP.
 */
#include "HomecareModel.hpp"

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

#include "global.hpp"                     // for RandomGenerator, make_unique
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator!=, operator<<
#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions

#include "solver/abstract-problem/Action.hpp"            // for Action
#include "solver/abstract-problem/Model.hpp"             // for Model::StepResult, Model
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/abstract-problem/State.hpp"             // for State, operator<<, operator==

#include "solver/changes/ChangeFlags.hpp"        // for ChangeFlags

#include "solver/indexing/FlaggingVisitor.hpp"
#include "solver/indexing/RTree.hpp"
#include "solver/indexing/SpatialIndexVisitor.hpp"             // for State, operator<<, operator==

#include "solver/mappings/actions/enumerated_actions.hpp"
#include "solver/mappings/observations/discrete_observations.hpp"

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"
#include "solver/StatePool.hpp"

#include "HomecareAction.hpp"
#include "HomecareObservation.hpp"
#include "HomecareOptions.hpp"
#include "HomecareState.hpp"                 // for HomecareState
#include "HomecareTextSerializer.hpp"

using std::cout;
using std::endl;

namespace homecare {

HomecareModel::HomecareModel(RandomGenerator *randGen, std::unique_ptr<HomecareOptions> options) :
            ModelWithProgramOptions("Homecare", randGen, std::move(options)),
            options_(const_cast<HomecareOptions *>(static_cast<HomecareOptions const *>(getOptions()))),
            moveCost_(options_->moveCost),
            diaMoveCost_(sqrt(2) * moveCost_),
            helpReward_(options_->helpReward),
            targetWStayProbability_(options_->targetWStayProbability),
            targetStayProbability_(options_->targetStayProbability),
            moveAccuracy_(options_->moveAccuracy),
            regionSensorAccuracy_(options_->regionSensorAccuracy),
            callProbability_(options_->callProbability),
            continueCallProbability_(options_->continueCallProbability),
            nRows_(0), // to be updated
            nCols_(0), // to be updated
            pathMapText_(),
            typeMapText_(),
            pathMap_(),
            typeMap_(),
            wCells_(),
            tCells_(),
            sCells_(),
            pCells_(),
            nActions_(9) {
    options_->numberOfStateVariables = 5;
    options_->minVal = -diaMoveCost_ / 1 - options_->discountFactor;
    options_->maxVal = helpReward_;

    // Read the map from the file.
    pathMapText_ = readMapText(options_->pathMapFilename);
    typeMapText_ = readMapText(options_->typeMapFilename);

    initialize();
    if (options_->hasVerboseOutput) {
        cout << "Constructed the HomecareModel" << endl;
        cout << "Discount: " << options_->discountFactor << endl;
        cout << "Size: " << nRows_ << " by " << nCols_ << endl;
        cout << "nActions: " << nActions_ << endl;
        cout << "nStVars: " << options_->numberOfStateVariables << endl;
        cout << "minParticleCount: " << options_->minParticleCount << endl;
        cout << "Environment:" << endl << endl;
        drawEnv(cout);
    }
}

std::vector<std::string> HomecareModel::readMapText(std::string filename) {
    std::vector<std::string> mapText;
    std::ifstream inFile;
    inFile.open(filename);
    if (!inFile.is_open()) {
        std::ostringstream message;
        message << "ERROR: Failed to open " << filename;
        debug::show_message(message.str());
        std::exit(1);
    }
    inFile >> nRows_ >> nCols_;
    std::string tmp;
    getline(inFile, tmp);
    for (long i = 0; i < nRows_; i++) {
        getline(inFile, tmp);
        mapText.push_back(tmp);
    }
    inFile.close();
    return mapText;
}

void HomecareModel::initialize() {
    GridPosition p;
    pathMap_.resize(nRows_);
    typeMap_.resize(nRows_);
    tCells_.clear();
    wCells_.clear();
    sCells_.clear();
    pCells_.clear();
    for (p.i = nRows_ - 1; p.i >= 0; p.i--) {
        pathMap_[p.i].resize(nCols_);
        typeMap_[p.i].resize(nCols_);
        for (p.j = 0; p.j < nCols_; p.j++) {
            char pathC = pathMapText_[p.i][p.j];
            HomecarePathCell pathCell;

            switch (pathC) {
                case '0':
                    pathCell = HomecarePathCell::EMPTY;
                    break;
                case 'u':
                    pathCell = HomecarePathCell::UP;
                    break;
                case 'r':
                    pathCell = HomecarePathCell::RIGHT;
                    break;
                case 'd':
                    pathCell = HomecarePathCell::DOWN;
                    break;
                case 'l':
                    pathCell = HomecarePathCell::LEFT;
                    break;
                case 'p':
                    pathCell = HomecarePathCell::UP_OR_RIGHT;
                    break;
                case 'm':
                    pathCell = HomecarePathCell::DOWN_OR_RIGHT;
                    break;
                case 'n':
                    pathCell = HomecarePathCell::DOWN_OR_LEFT;
                    break;
                case 'o':
                    pathCell = HomecarePathCell::UP_OR_LEFT;
                    break;
                case '1':
                    pathCell = HomecarePathCell::WALL;
                    break;
                default:
                    cout << "ERROR reading path map, invalid char " << pathC << endl;
                    pathCell = HomecarePathCell::EMPTY;
            } 
            pathMap_[p.i][p.j] = pathCell;
            if (pathCell != HomecarePathCell::EMPTY && 
            		pathCell != HomecarePathCell::WALL) {
                pCells_.push_back(p);
            }
            
            char typeC = typeMapText_[p.i][p.j];
            HomecareTypeCell typeCell;

            switch (typeC) {
                case 't':
                    typeCell = HomecareTypeCell::TARGET_START;
                    tCells_.push_back(p);
                    break;
                case 'w':
                    typeCell = HomecareTypeCell::WASHROOM;
                    wCells_.push_back(p);
                    break;
                case 's':
                    typeCell = HomecareTypeCell::START;
                    sCells_.push_back(p);
                    break;
                default:
                    typeCell = HomecareTypeCell::OTHER;
            }   
            typeMap_[p.i][p.j] = typeCell;
        }
    }
}

GridPosition HomecareModel::randomTCell() {
    return tCells_[std::uniform_int_distribution<long>(
        0, tCells_.size() - 1)(*getRandomGenerator())];
}

GridPosition HomecareModel::randomSCell() {
    return sCells_[std::uniform_int_distribution<long>(
        0, sCells_.size() - 1)(*getRandomGenerator())];
}

GridPosition HomecareModel::randomPCell() {
    return pCells_[std::uniform_int_distribution<long>(
        0, pCells_.size() - 1)(*getRandomGenerator())];
}

GridPosition HomecareModel::randomNotWallCell() {
    GridPosition pos;
    while (true) {
        pos.i = std::uniform_int_distribution<long>(0, nRows_ - 1)(
                *getRandomGenerator());
        pos.j = std::uniform_int_distribution<long>(0, nCols_ - 1)(
                *getRandomGenerator());
        if (pathMap_[pos.i][pos.j] != HomecarePathCell::WALL) {
            break;
        }
    }
    return pos;
}

/* --------------- The model interface proper ----------------- */
std::unique_ptr<solver::State> HomecareModel::sampleAnInitState() {
    GridPosition robotPos = randomSCell();
    GridPosition targetPos = randomTCell();
    return std::make_unique<HomecareState>(robotPos, targetPos, false);
}

std::unique_ptr<solver::State> HomecareModel::sampleStateUninformed() {
    GridPosition robotPos = randomNotWallCell();
    GridPosition targetPos = randomPCell();
    bool call = std::bernoulli_distribution(callProbability_)(*getRandomGenerator());
    return std::make_unique<HomecareState>(robotPos, targetPos, call);
}

bool HomecareModel::isTerminal(solver::State const &state) {
    return false;
}


/* -------------------- Black box dynamics ---------------------- */
std::pair<std::unique_ptr<HomecareState>, bool> HomecareModel::makeNextState(
        solver::State const &state, solver::Action const &action) {
    HomecareState const &homecareState = static_cast<HomecareState const &>(state);
    HomecareAction const &homecareAction = static_cast<HomecareAction const &>(action);

    GridPosition robotPos = homecareState.getRobotPos();
    GridPosition targetPos = homecareState.getTargetPos();
    bool call = homecareState.getCall();
    GridPosition newTargetPos = targetPos;
    if (!homecareState.getCall()) {
        newTargetPos = sampleMovedTargetPosition(targetPos);
    }
    GridPosition newRobotPos;
    bool wasValid;
    ActionType actionType = homecareAction.getActionType();
    if (actionType == ActionType::WAIT) {
    	newRobotPos = robotPos;
    	wasValid = true;
	} else {
	    std::tie(newRobotPos, wasValid) = sampleMovedRobotPosition(
	            robotPos, homecareAction.getActionType());
	}
    bool newCall = updateCall(newRobotPos, newTargetPos, call);
    return std::make_pair(std::make_unique<HomecareState>(
        newRobotPos, newTargetPos, newCall), wasValid);
}

std::vector<GridPosition> HomecareModel::validMovedTargetPositions(
        GridPosition const &targetPos) {
    std::vector<GridPosition> positions;

    HomecarePathCell c = pathMap_[targetPos.i][targetPos.j];
    switch(c) {
        case HomecarePathCell::UP:
            positions.push_back(GridPosition(targetPos.i - 1, targetPos.j));
            break;
        case HomecarePathCell::RIGHT:
            positions.push_back(GridPosition(targetPos.i, targetPos.j + 1));
            break;
        case HomecarePathCell::DOWN:
            positions.push_back(GridPosition(targetPos.i + 1, targetPos.j));
            break;
        case HomecarePathCell::LEFT:
            positions.push_back(GridPosition(targetPos.i, targetPos.j - 1));
            break;
        case HomecarePathCell::UP_OR_RIGHT:
            positions.push_back(GridPosition(targetPos.i - 1, targetPos.j));
            positions.push_back(GridPosition(targetPos.i, targetPos.j + 1));
            break;
        case HomecarePathCell::DOWN_OR_RIGHT:
            positions.push_back(GridPosition(targetPos.i + 1, targetPos.j));
            positions.push_back(GridPosition(targetPos.i, targetPos.j + 1));
            break;
        case HomecarePathCell::DOWN_OR_LEFT:
            positions.push_back(GridPosition(targetPos.i + 1, targetPos.j));
            positions.push_back(GridPosition(targetPos.i, targetPos.j - 1));
            break;
        case HomecarePathCell::UP_OR_LEFT:
            positions.push_back(GridPosition(targetPos.i - 1, targetPos.j));
            positions.push_back(GridPosition(targetPos.i, targetPos.j - 1));
            break;
        default:
            cout << "ERROR: Target not on path" << endl;
    }

    return positions;
}

std::unordered_map<GridPosition, double> HomecareModel::getNextRobotPositionDistribution(
        GridPosition const &robotPos, ActionType action) {
    std::unordered_map<GridPosition, double> distribution; 

    // No change in position for WAIT action
    if (action == ActionType::WAIT) {
    	distribution[robotPos] = 1.0;
    	return distribution;
    }

    // Add position for no deviation
    GridPosition noDevPos;
    bool valid;
    std::tie(noDevPos, valid) = getMovedPos(robotPos, action);
    if (!valid) {
    	noDevPos = robotPos;
    }
    distribution[noDevPos] = moveAccuracy_;

    // Add positions for deviations
    ActionType clockwise = shiftClockwise(action);
    ActionType aClockwise = shiftAntiClockwise(action);
    GridPosition devPos1, devPos2;
    std::tie(devPos1, valid) = getMovedPos(robotPos, clockwise);
    if (!valid) {
    	distribution[robotPos] += 0.5 * (1 - moveAccuracy_);
    } else {
    	distribution[devPos1] = 0.5 * (1 - moveAccuracy_);
    }
    std::tie(devPos2, valid) = getMovedPos(robotPos, aClockwise);
    if (!valid) {
    	distribution[robotPos] += 0.5 * (1 - moveAccuracy_);
    } else {
    	distribution[devPos2] = 0.5 * (1 - moveAccuracy_);
    }
    return distribution;
}

/** Generates a proper distribution for next target positions. */
std::unordered_map<GridPosition, double> HomecareModel::getNextTargetPositionDistribution(
        GridPosition const &targetPos, bool call) {
    std::unordered_map<GridPosition, double> distribution;

    // If target is calling for assistance, it does not move
    if (call) {
        distribution[targetPos] = 1.0;
        return std::move(distribution);
    }

    std::vector<GridPosition> movePositions = validMovedTargetPositions(targetPos);
    double movedPosProb;
    if (typeMap_[targetPos.i][targetPos.j] == HomecareTypeCell::WASHROOM) {
        distribution[targetPos] += targetWStayProbability_;
        movedPosProb = (1 - targetWStayProbability_) / movePositions.size();
    } else {
        distribution[targetPos] += targetStayProbability_;
        movedPosProb = (1 - targetStayProbability_) / movePositions.size();
    }
    for (GridPosition p : movePositions) {
        distribution[p] += movedPosProb;
    }
    return std::move(distribution);
}

std::unordered_map<bool, double> HomecareModel::getNextCallDistribution(
        GridPosition const &robotPos, GridPosition const &targetPos, bool call) {
	std::unordered_map<bool, double> distribution;
	if (call && robotPos == targetPos) {
		distribution[false] = 1.0;
	} else if (call) {
		distribution[false] = 1.0 - continueCallProbability_;
		distribution[true] = continueCallProbability_;
	} else {
		distribution[false] = 1.0 - callProbability_;
		distribution[true] = callProbability_;
	}
	return distribution;
}

std::pair<GridPosition, bool> HomecareModel::sampleMovedRobotPosition(
        GridPosition const &robotPos, ActionType action) {
    if (std::bernoulli_distribution(moveAccuracy_)(*getRandomGenerator())) {
        return getMovedPos(robotPos, action);
    }
    if (std::bernoulli_distribution(0.5)(*getRandomGenerator())) {
        action = shiftAntiClockwise(action);
    } else {
        action = shiftClockwise(action);
    }
    return getMovedPos(robotPos, action);
}

GridPosition HomecareModel::sampleMovedTargetPosition(GridPosition const &targetPos) {
    if (std::bernoulli_distribution(targetStayProbability_)(
            *getRandomGenerator())) {
        return targetPos;
    }
    std::vector<GridPosition> movePositions = validMovedTargetPositions(targetPos);
    return movePositions[std::uniform_int_distribution<long>(0,
            movePositions.size() - 1)(*getRandomGenerator())];
}

std::pair<GridPosition, bool> HomecareModel::getMovedPos(GridPosition const &position,
        ActionType action) {
    GridPosition movedPos = position;
    switch (action) {
	    case ActionType::NORTH:
	        movedPos.i -= 1;
	        break;
	    case ActionType::NORTH_EAST:
	        movedPos.i -= 1;
	        movedPos.j += 1;
	        break;
	    case ActionType::EAST:
	        movedPos.j += 1;
	        break;
	    case ActionType::SOUTH_EAST:
	        movedPos.i += 1;
	        movedPos.j += 1;
	        break;
	    case ActionType::SOUTH:
	        movedPos.i += 1;
	        break;
	    case ActionType::SOUTH_WEST:
	        movedPos.i += 1;
	        movedPos.j -= 1;
	        break;
	    case ActionType::WEST:
	        movedPos.j -= 1;
	        break;
	    case ActionType::NORTH_WEST:
	        movedPos.i -= 1;
	        movedPos.j -= 1;
	        break;
	    case ActionType::WAIT:
	        break;
	    default:
	        std::ostringstream message;
	        message << "Invalid action: " << (long) action;
	        debug::show_message(message.str());
	        break;
    }
    bool wasValid = isValid(movedPos);
    if (!wasValid) {
        movedPos = position;
    }
    return std::make_pair(movedPos, wasValid);
}

bool HomecareModel::isValid(GridPosition const &position) {
    return (position.i >= 0 && position.i < nRows_ && position.j >= 0
            && position.j < nCols_ && pathMap_[position.i][position.j] != HomecarePathCell::WALL);
}

ActionType HomecareModel::shiftAntiClockwise(ActionType action) {
	int result = (int) action - 1;
	if (result < 0) {
		result = 7;
	}
	return (ActionType) result;
}

ActionType HomecareModel::shiftClockwise(ActionType action) {
	int result = (int) action + 1;
	if (result > 7) {
		result = 0;
	}
	return (ActionType) result;
}

bool HomecareModel::updateCall(GridPosition robotPos, GridPosition targetPos, bool call) {
    if (call && robotPos == targetPos) {
        return false;
    } else if (call) {
        return std::bernoulli_distribution(continueCallProbability_)(
            *getRandomGenerator());
    } else {
        return std::bernoulli_distribution(callProbability_)(
            *getRandomGenerator());
    }
}

std::unique_ptr<solver::Observation> HomecareModel::makeObservation(
        HomecareState const &nextState) {
    GridPosition robotPos = nextState.getRobotPos();
    GridPosition targetPos(-1, -1);
    if (nextState.getTargetPos().euclideanDistanceTo(robotPos) < 1.5) {
        targetPos = nextState.getTargetPos();
    }
    int targetRegion = sampleObservationRegion(nextState);
    return std::make_unique<HomecareObservation>(
        robotPos, targetPos, targetRegion, nextState.getCall());
}

int HomecareModel::sampleObservationRegion(HomecareState const &state) {
    GridPosition tPos = state.getTargetPos();
    int currentRegion;
    int a = std::uniform_int_distribution<int>(0, 2)(*getRandomGenerator());
    int other;
    if (tPos.i <= nRows_ / 2) {
        if (tPos.j <= nCols_ / 2) {
            currentRegion = 0;
            int otherRegions[] = {1, 2, 3};
            other = otherRegions[a];
        } else {
            currentRegion = 1;
            int otherRegions[] = {0, 2, 3};
            other = otherRegions[a];
        }
    } else {
        if (tPos.j <= nCols_ / 2) {
            currentRegion = 2;
            int otherRegions[] = {0, 1, 3};
            other = otherRegions[a];
        } else {
            currentRegion = 3;
            int otherRegions[] = {0, 1, 2};
            other = otherRegions[a];
        }
    }
    if (std::bernoulli_distribution(regionSensorAccuracy_)(
            *getRandomGenerator())) {
        return currentRegion;
    }
    return other;
}

double HomecareModel::generateReward(solver::State const &state,
        solver::Action const &action,
        solver::TransitionParameters const */*tp*/,
        solver::State const *nextState) {
    double reward = 0;
    ActionType aType = static_cast<HomecareAction const &>(action).getActionType();
    if (aType == ActionType::NORTH || aType == ActionType::EAST ||
            aType == ActionType::SOUTH || aType == ActionType::WEST) {
        reward -= moveCost_;
    } else if (aType != ActionType::WAIT) {
        reward -= diaMoveCost_;
    }
    if (nextState) {
        HomecareState const &s = static_cast<HomecareState const &>(state);
        HomecareState const *ns = static_cast<HomecareState const *>(nextState);
        if (s.getCall() && ns->getRobotPos() == ns->getTargetPos()) {
            reward += helpReward_;
        }
    }
    return reward;
}

std::unique_ptr<solver::State> HomecareModel::generateNextState(
        solver::State const &state, solver::Action const &action,
        solver::TransitionParameters const */*tp*/) {
    return makeNextState(static_cast<HomecareState const &>(state), action).first;
}

std::unique_ptr<solver::Observation> HomecareModel::generateObservation(
        solver::State const */*state*/, solver::Action const &/*action*/,
        solver::TransitionParameters const */*tp*/,
        solver::State const &nextState) {
    return makeObservation(static_cast<HomecareState const &>(nextState));
}

solver::Model::StepResult HomecareModel::generateStep(solver::State const &state,
        solver::Action const &action) {
    solver::Model::StepResult result;
    result.action = action.copy();
    std::unique_ptr<HomecareState> nextState = makeNextState(state, action).first;

    result.observation = makeObservation(*nextState);
    result.reward = generateReward(state, action, nullptr, nextState.get());
    result.isTerminal = isTerminal(*nextState);
    result.nextState = std::move(nextState);
    return result;
}


/* -------------- Methods for handling model changes ---------------- */
void HomecareModel::applyChanges(std::vector<std::unique_ptr<solver::ModelChange>> const &changes,
        solver::Solver *solver) {
    
    solver::StatePool *pool = nullptr;
    if (solver != nullptr) {
        pool = solver->getStatePool();
    }

    solver::HeuristicFunction heuristic = getHeuristicFunction();
    std::vector<double> allHeuristicValues;
    if (pool != nullptr) {
        long nStates = pool->getNumberOfStates();
        allHeuristicValues.resize(nStates);
        for (long index = 0; index < nStates; index++) {
            allHeuristicValues[index] = heuristic(nullptr, pool->getInfoById(index)->getState(),
                    nullptr);
        }
    }

    for (auto const &change : changes) {
        HomecareChange const &homecareChange = static_cast<HomecareChange const &>(*change);
        if (options_->hasVerboseOutput) {
            cout << homecareChange.changeType << " " << homecareChange.i0 << " "
                    << homecareChange.j0;
            cout << " " << homecareChange.i1 << " " << homecareChange.j1 << endl;
        }

        HomecareTypeCell newCellType;
        if (homecareChange.changeType == "Add W") {
            newCellType = HomecareTypeCell::WASHROOM;
        } else {
            cout << "Invalid change type: " << homecareChange.changeType;
            continue;
        }

        for (long i = homecareChange.i0; i <= homecareChange.i1; i++) {
            for (long j = homecareChange.j0; j <= homecareChange.j1; j++) {
                typeMap_[i][j] = newCellType;
                wCells_.push_back(GridPosition(i, j));
            }
        }

        if (pool == nullptr) {
            continue;
        }

        solver::RTree *tree = static_cast<solver::RTree *>(pool->getStateIndex());

        double iLo = homecareChange.i0;
        double iHi = homecareChange.i1;
        double iMx = nRows_ - 1.0;

        double jLo = homecareChange.j0;
        double jHi = homecareChange.j1;
        double jMx = nCols_ - 1.0;

        // Revise state transitions
        solver::FlaggingVisitor visitor(pool, solver::ChangeFlags::TRANSITION);
        tree->boxQuery(visitor,
                {0.0, 0.0, iLo - 1, jLo - 1, 0.0},
                {iMx, jMx, iHi + 1, jHi + 1, 0.0});
    }

    // Check for heuristic changes.
    if (pool != nullptr) {
        long nStates = pool->getNumberOfStates();
        for (long index = 0; index < nStates; index++) {
            double oldValue = allHeuristicValues[index];
            solver::StateInfo *info = pool->getInfoById(index);
            double newValue = heuristic(nullptr, info->getState(), nullptr);
            if (std::abs(newValue - oldValue) > 1e-5) {
                pool->setChangeFlags(info, solver::ChangeFlags::HEURISTIC);
            }
        }
    }
}


/* ------------ Methods for handling particle depletion -------------- */
std::vector<std::unique_ptr<solver::State>> HomecareModel::generateParticles(
        solver::BeliefNode */*previousBelief*/, solver::Action const &action,
        solver::Observation const &obs,
        long nParticles,
        std::vector<solver::State const *> const &previousParticles) {
    std::vector<std::unique_ptr<solver::State>> newParticles;
    HomecareObservation const &observation =
            (static_cast<HomecareObservation const &>(obs));
    ActionType actionType =
            (static_cast<HomecareAction const &>(action).getActionType());

    typedef std::unordered_map<HomecareState, double> WeightMap;
    WeightMap weights;
    double weightTotal = 0;

    GridPosition newRobotPos(observation.getRobotPos());
    for (solver::State const *state : previousParticles) {
        HomecareState const *homecareState = static_cast<HomecareState const *>(state);
        GridPosition oldRobotPos(homecareState->getRobotPos());

        // Get probability distribution for robot moves
        std::unordered_map<GridPosition, double> robotPosDistribution = (
                getNextRobotPositionDistribution(oldRobotPos, actionType));

        // Ignore states that do not match knowledge of the robot's position.
        bool matchFound = false;
        for (auto const &entry : robotPosDistribution) {
        	if (newRobotPos == entry.first) {
        		matchFound = true;
        		break;
        	}
        }
        if (!matchFound) {
            continue;
        }

        // Generate new particles
        bool oldCall = homecareState->getCall();
        GridPosition oldTargetPos(homecareState->getTargetPos());
        std::unordered_map<GridPosition, double> targetPosDistribution = (
                getNextTargetPositionDistribution(oldTargetPos, oldCall));
        for (auto const &robotEntry : robotPosDistribution) {
		    for (auto const &targetEntry : targetPosDistribution) {
		    	std::unordered_map<bool, double> callDistribution = (
        				getNextCallDistribution(robotEntry.first, targetEntry.first, oldCall));
			    for (auto const &callEntry : callDistribution) {
			    	HomecareState newState(robotEntry.first, targetEntry.first, callEntry.first);
		            weights[newState] += robotEntry.second * targetEntry.second * callEntry.second;
		            weightTotal += robotEntry.second * targetEntry.second * callEntry.second;
		        }
	        }
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
            newParticles.push_back(std::make_unique<HomecareState>(it->first));
        }
    }

    return newParticles;
}

std::vector<std::unique_ptr<solver::State>> HomecareModel::generateParticles(
        solver::BeliefNode */*previousBelief*/, solver::Action const &action,
        solver::Observation const &obs, long nParticles) {
    std::vector<std::unique_ptr<solver::State>> newParticles;
    HomecareObservation const &observation =
            (static_cast<HomecareObservation const &>(obs));
    ActionType actionType =
            (static_cast<HomecareAction const &>(action).getActionType());
    GridPosition newRobotPos(observation.getRobotPos());

    while ((long)newParticles.size() < nParticles) {
        std::unique_ptr<solver::State> state = sampleStateUninformed();
        solver::Model::StepResult result = generateStep(*state, action);
        if (obs == *result.observation) {
            newParticles.push_back(std::move(result.nextState));
        }
    }
    return newParticles;
}


/* --------------- Pretty printing methods ----------------- */

void HomecareModel::drawEnv(std::ostream &os) {
    for (std::size_t i = 0; i < pathMap_.size(); i++) {
        for (std::size_t j = 0; j < pathMap_[0].size(); j++) {
            switch (pathMap_[i][j]) {
                case HomecarePathCell::EMPTY:
                    os << " 0";
                    break;
                case HomecarePathCell::UP:
                    os << " u";
                    break;
                case HomecarePathCell::RIGHT:
                    os << " r";
                    break;
                case HomecarePathCell::DOWN:
                    os << " d";
                    break;
                case HomecarePathCell::LEFT:
                    os << " l";
                    break;
                case HomecarePathCell::UP_OR_RIGHT:
                    os << "ur";
                    break;
                case HomecarePathCell::DOWN_OR_RIGHT:
                    os << "dr";
                    break;
                case HomecarePathCell::DOWN_OR_LEFT:
                    os << "dl";
                    break;
                case HomecarePathCell::UP_OR_LEFT:
                    os << "ul";
                    break;
                case HomecarePathCell::WALL:
                    os << "XX";
                    break;
                default:
                    os << "ER";
            }
        }
        os << endl;
    }
    os << endl;
    for (std::size_t i = 0; i < typeMap_.size(); i++) {
        for (std::size_t j = 0; j < typeMap_[0].size(); j++) {
            switch (typeMap_[i][j]) {
                case HomecareTypeCell::TARGET_START:
                    os << " t";
                    break;
                case HomecareTypeCell::WASHROOM:
                    os << " w";
                    break;
                case HomecareTypeCell::START:
                    os << " s";
                    break;
                case HomecareTypeCell::OTHER:
                    os << " o";
                    break;
                default:
                    os << "ER";
            }
        }
        os << endl;
    }
}

void HomecareModel::drawSimulationState(solver::BeliefNode const *belief,
        solver::State const &state, std::ostream &os) {
    HomecareState const &homecareState = static_cast<HomecareState const &>(state);
    std::vector<solver::State const *> particles = belief->getStates();
    std::vector<std::vector<long>> particleCounts(nRows_,
            std::vector<long>(nCols_));
    for (solver::State const *particle : particles) {
        GridPosition targetPos =
                static_cast<HomecareState const &>(*particle).getTargetPos();
        particleCounts[targetPos.i][targetPos.j] += 1;
    }
    long maxParticleCount = 0;
    for (std::size_t i = 0; i < pathMap_.size(); i++) {
        for (std::size_t j = 0; j < pathMap_[0].size(); j++) {
            if (particleCounts[i][j] > maxParticleCount) {
                maxParticleCount = particleCounts[i][j];
            }
        }
    }

    std::vector<int> colors { 196, 161, 126, 91, 56, 21, 26, 31, 36, 41, 46 };
    if (options_->hasColorOutput) {
        os << "Color map: ";
        for (int color : colors) {
            os << "\033[38;5;" << color << "m";
            os << '*';
            os << "\033[0m";
        }
        os << endl;
    }
    for (std::size_t i = 0; i < pathMap_.size(); i++) {
        for (std::size_t j = 0; j < pathMap_[0].size(); j++) {
            double proportion = (double) particleCounts[i][j]
                    / maxParticleCount;
            if (options_->hasColorOutput) {
                if (proportion > 0) {
                    int color = colors[proportion * (colors.size() - 1)];
                    os << "\033[38;5;" << color << "m";
                }
            }
            GridPosition pos(i, j);
            bool hasRobot = (pos == homecareState.getRobotPos());
            bool hasTarget = (pos == homecareState.getTargetPos());
            if (hasRobot) {
                if (hasTarget) {
                    os << "#";
                } else {
                    os << "r";
                }
            } else if (hasTarget) {
                os << "T";
            } else {
                if (pathMap_[i][j] == HomecarePathCell::WALL) {
                    os << "X";
                } else if (typeMap_[i][j] == HomecareTypeCell::WASHROOM) {
                    os << "w";
                } else {
                    os << ".";
                }
            }
            if (options_->hasColorOutput) {
                os << "\033[0m";
            }
        }
        os << endl;
    }
    
}


/* ---------------------- Basic customizations  ---------------------- */
double HomecareModel::getDefaultHeuristicValue(solver::HistoryEntry const */*entry*/,
            solver::State const *state, solver::HistoricalData const */*data*/) {
    HomecareState const &homecareState = static_cast<HomecareState const &>(*state);
    GridPosition robotPos = homecareState.getRobotPos();
    GridPosition targetPos = homecareState.getTargetPos();
    long dist = robotPos.manhattanDistanceTo(targetPos);
    double nSteps = dist / targetStayProbability_;
    double finalDiscount = std::pow(options_->discountFactor, nSteps);
    double qVal = -moveCost_ * (1 - finalDiscount) / (1 - options_->discountFactor);
    if (homecareState.getCall()) {
        qVal += finalDiscount * helpReward_;
    }
    return qVal;
}


/* ------- Customization of more complex solver functionality  --------- */
std::vector<std::unique_ptr<solver::DiscretizedPoint>> HomecareModel::getAllActionsInOrder() {
    std::vector<std::unique_ptr<solver::DiscretizedPoint>> allActions;
    for (long code = 0; code < nActions_; code++) {
        allActions.push_back(std::make_unique<HomecareAction>(code));
    }
    return allActions;
}

std::unique_ptr<solver::ActionPool> HomecareModel::createActionPool(solver::Solver */*solver*/) {
    return std::make_unique<solver::EnumeratedActionPool>(this, getAllActionsInOrder());
}
std::unique_ptr<solver::Serializer> HomecareModel::createSerializer(solver::Solver *solver) {
    return std::make_unique<HomecareTextSerializer>(solver);
}
} /* namespace homecare */
