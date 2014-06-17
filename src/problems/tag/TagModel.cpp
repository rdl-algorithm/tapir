#include "TagModel.hpp"

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

#include "solver/mappings/actions/enumerated_actions.hpp"
#include "solver/mappings/observations/discrete_observations.hpp"

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"
#include "solver/StatePool.hpp"

#include "TagAction.hpp"
#include "TagObservation.hpp"
#include "TagState.hpp"                 // for TagState
#include "TagTextSerializer.hpp"

using std::cout;
using std::endl;
namespace po = boost::program_options;

namespace tag {
TagModel::TagModel(RandomGenerator *randGen, po::variables_map vm) :
            ModelWithProgramOptions(randGen, vm),
            moveCost_(vm["problem.moveCost"].as<double>()),
            tagReward_(vm["problem.tagReward"].as<double>()),
            failedTagPenalty_(vm["problem.failedTagPenalty"].as<double>()),
            opponentStayProbability_(vm["problem.opponentStayProbability"].as<double>()),
            nRows_(0), // to be updated
            nCols_(0), // to be updated
            mapText_(), // will be pushed to
            envMap_(), // will be pushed to
            nActions_(5),
            nStVars_(5),
            minVal_(-failedTagPenalty_ / (1 - getDiscountFactor())),
            maxVal_(tagReward_) {
    // Read the map from the file.
    std::ifstream inFile;
    char const *mapPath = vm["problem.mapPath"].as<std::string>().c_str();
    inFile.open(mapPath);
    if (!inFile.is_open()) {
        std::ostringstream message;
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
    if (hasVerboseOutput()) {
        cout << "Constructed the TagModel" << endl;
        cout << "Discount: " << getDiscountFactor() << endl;
        cout << "Size: " << nRows_ << " by " << nCols_ << endl;
        cout << "move cost: " << moveCost_ << endl;
        cout << "nActions: " << nActions_ << endl;
        cout << "nStVars: " << nStVars_ << endl;
        cout << "minParticleCount: " << getMinParticleCount() << endl;
        cout << "Environment:" << endl << endl;
        drawEnv(cout);
    }
}

void TagModel::initialize() {
    GridPosition p;
    envMap_.resize(nRows_);
    for (p.i = nRows_ - 1; p.i >= 0; p.i--) {
        envMap_[p.i].resize(nCols_);
        for (p.j = 0; p.j < nCols_; p.j++) {
            char c = mapText_[p.i][p.j];
            TagCellType cellType;
            if (c == 'X') {
                cellType = WALL;
            } else {
                cellType = TagCellType::EMPTY;
            }
            envMap_[p.i][p.j] = cellType;
        }
    }
}

GridPosition TagModel::randomEmptyCell() {
    GridPosition pos;
    while (true) {
        pos.i = std::uniform_int_distribution<long>(0, nRows_ - 1)(
                *getRandomGenerator());
        pos.j = std::uniform_int_distribution<long>(0, nCols_ - 1)(
                *getRandomGenerator());
        if (envMap_[pos.i][pos.j] == TagCellType::EMPTY) {
            break;
        }
    }
    return pos;
}


/* --------------- The model interface proper ----------------- */
std::unique_ptr<solver::State> TagModel::sampleAnInitState() {
    return sampleStateUniform();
}

std::unique_ptr<solver::State> TagModel::sampleStateUniform() {
    GridPosition robotPos = randomEmptyCell();
    GridPosition opponentPos = randomEmptyCell();
    return std::make_unique<TagState>(robotPos, opponentPos, false);
}

bool TagModel::isTerminal(solver::State const &state) {
    return static_cast<TagState const &>(state).isTagged();
}


/* -------------------- Black box dynamics ---------------------- */
std::pair<std::unique_ptr<TagState>, bool> TagModel::makeNextState(
        solver::State const &state, solver::Action const &action) {
    TagState const &tagState = static_cast<TagState const &>(state);
    if (tagState.isTagged()) {
        return std::make_pair(std::make_unique<TagState>(tagState), false);
    }

    TagAction const &tagAction = static_cast<TagAction const &>(action);

    GridPosition robotPos = tagState.getRobotPosition();
    GridPosition opponentPos = tagState.getOpponentPosition();
    if (tagAction.getActionType() == ActionType::TAG
            && robotPos == opponentPos) {
        return std::make_pair(
                std::make_unique<TagState>(robotPos, opponentPos, true), true);
    }

    GridPosition newOpponentPos = getMovedOpponentPos(robotPos, opponentPos);
    GridPosition newRobotPos;
    bool wasValid;
    std::tie(newRobotPos, wasValid) = getMovedPos(robotPos, tagAction.getActionType());
    return std::make_pair(std::make_unique<TagState>(newRobotPos, newOpponentPos, false),
            wasValid);
}

std::vector<ActionType> TagModel::makeOpponentActions(
        GridPosition const &robotPos, GridPosition const &opponentPos) {
    std::vector<ActionType> actions;
    if (robotPos.i > opponentPos.i) {
        actions.push_back(ActionType::NORTH);
        actions.push_back(ActionType::NORTH);
    } else if (robotPos.i < opponentPos.i) {
        actions.push_back(ActionType::SOUTH);
        actions.push_back(ActionType::SOUTH);
    } else {
        actions.push_back(ActionType::NORTH);
        actions.push_back(ActionType::SOUTH);
    }
    if (robotPos.j > opponentPos.j) {
        actions.push_back(ActionType::WEST);
        actions.push_back(ActionType::WEST);
    } else if (robotPos.j < opponentPos.j) {
        actions.push_back(ActionType::EAST);
        actions.push_back(ActionType::EAST);
    } else {
        actions.push_back(ActionType::EAST);
        actions.push_back(ActionType::WEST);
    }
    return actions;
}

GridPosition TagModel::getMovedOpponentPos(GridPosition const &robotPos,
        GridPosition const &opponentPos) {
    // Randomize to see if the opponent stays still.
    if (std::bernoulli_distribution(opponentStayProbability_)(
            *getRandomGenerator())) {
        return opponentPos;
    }
    std::vector<ActionType> actions(makeOpponentActions(robotPos, opponentPos));
    ActionType action = actions[std::uniform_int_distribution<long>(0,
            actions.size() - 1)(*getRandomGenerator())];
    return getMovedPos(opponentPos, action).first;
}

std::pair<GridPosition, bool> TagModel::getMovedPos(GridPosition const &position,
        ActionType action) {
    GridPosition movedPos = position;
    switch (action) {
    case ActionType::NORTH:
        movedPos.i -= 1;
        break;
    case ActionType::EAST:
        movedPos.j += 1;
        break;
    case ActionType::SOUTH:
        movedPos.i += 1;
        break;
    case ActionType::WEST:
        movedPos.j -= 1;
        break;
    case ActionType::TAG:
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

bool TagModel::isValid(GridPosition const &position) {
    return (position.i >= 0 && position.i < nRows_ && position.j >= 0
            && position.j < nCols_ && envMap_[position.i][position.j] != WALL);
}

std::unique_ptr<solver::Observation> TagModel::makeObservation(
        solver::Action const & /*action*/, TagState const &nextState) {
    return std::make_unique<TagObservation>(nextState.getRobotPosition(),
            nextState.getRobotPosition() == nextState.getOpponentPosition());
}

double TagModel::generateReward(solver::State const &state,
        solver::Action const &action,
        solver::TransitionParameters const */*tp*/,
        solver::State const */*nextState*/) {
    if (static_cast<TagAction const &>(action).getActionType()
            == ActionType::TAG) {
        TagState const &tagState = static_cast<TagState const &>(state);
        if (tagState.getRobotPosition() == tagState.getOpponentPosition()) {
            return tagReward_;
        } else {
            return -failedTagPenalty_;
        }
    } else {
        return -moveCost_;
    }
}

std::unique_ptr<solver::State> TagModel::generateNextState(
        solver::State const &state, solver::Action const &action,
        solver::TransitionParameters const */*tp*/) {
    return makeNextState(static_cast<TagState const &>(state), action).first;
}

std::unique_ptr<solver::Observation> TagModel::generateObservation(
        solver::State const */*state*/, solver::Action const &action,
        solver::TransitionParameters const */*tp*/,
        solver::State const &nextState) {
    return makeObservation(action, static_cast<TagState const &>(nextState));
}

solver::Model::StepResult TagModel::generateStep(solver::State const &state,
        solver::Action const &action) {
    solver::Model::StepResult result;
    result.action = action.copy();
    std::unique_ptr<TagState> nextState = makeNextState(state, action).first;

    result.observation = makeObservation(action, *nextState);
    result.reward = generateReward(state, action, nullptr, nullptr);
    result.isTerminal = isTerminal(*nextState);
    result.nextState = std::move(nextState);
    return result;
}


/* -------------- Methods for handling model changes ---------------- */
void TagModel::applyChanges(std::vector<std::unique_ptr<solver::ModelChange>> const &changes,
        solver::Solver *solver) {
    solver::StatePool *pool = nullptr;
    if (solver != nullptr) {
        pool = solver->getStatePool();
    }

    for (auto const &change : changes) {
        TagChange const &tagChange = static_cast<TagChange const &>(*change);
        if (hasVerboseOutput()) {
            cout << tagChange.changeType << " " << tagChange.i0 << " "
                    << tagChange.j0;
            cout << " " << tagChange.i1 << " " << tagChange.j1 << endl;
        }

        TagCellType newCellType;
        if (tagChange.changeType == "Add Obstacles") {
            newCellType = TagCellType::WALL;
        } else if (tagChange.changeType == "Remove Obstacles") {
            newCellType = TagCellType::EMPTY;
        } else {
            cout << "Invalid change type: " << tagChange.changeType;
            continue;
        }

        for (long i = static_cast<long>(tagChange.i0); i <= tagChange.i1; i++) {
            for (long j = static_cast<long>(tagChange.j0); j <= tagChange.j1; j++) {
                envMap_[i][j] = newCellType;
            }
        }

        if (pool == nullptr) {
            continue;
        }

        solver::RTree *tree = static_cast<solver::RTree *>(pool->getStateIndex());

        double iLo = tagChange.i0;
        double iHi = tagChange.i1;
        double iMx = nRows_ - 1.0;

        double jLo = tagChange.j0;
        double jHi = tagChange.j1;
        double jMx = nCols_ - 1.0;

        // Adding walls => any states where the robot or the opponent are in a wall must
        // be deleted.
        if (newCellType == TagCellType::WALL) {
            solver::FlaggingVisitor visitor(pool, solver::ChangeFlags::DELETED);
            // Robot is in a wall.
            tree->boxQuery(visitor,
                    {iLo, jLo, 0.0, 0.0, 0.0},
                    {iHi, jHi, iMx, jMx, 1.0});
            // Opponent is in a wall.
            tree->boxQuery(visitor,
                    {0.0, 0.0, iLo, jLo, 0.0},
                    {iMx, jMx, iHi, jHi, 1.0});

        }

        // Also, state transitions around the edges of the new / former obstacle must be revised.
        solver::FlaggingVisitor visitor(pool, solver::ChangeFlags::TRANSITION);
        tree->boxQuery(visitor,
                {iLo - 1, jLo - 1, 0.0, 0.0, 0.0},
                {iHi + 1, jHi + 1, iMx, jMx, 1.0});
        tree->boxQuery(visitor,
                {0.0, 0.0, iLo - 1, jLo - 1, 0.0},
                {iMx, jMx, iHi + 1, jHi + 1, 1.0});
    }
}


/* ------------ Methods for handling particle depletion -------------- */
std::vector<std::unique_ptr<solver::State>> TagModel::generateParticles(
        solver::BeliefNode */*previousBelief*/, solver::Action const &action,
        solver::Observation const &obs,
        long nParticles,
        std::vector<solver::State const *> const &previousParticles) {
    std::vector<std::unique_ptr<solver::State>> newParticles;
    TagObservation const &observation =
            (static_cast<TagObservation const &>(obs));
    ActionType actionType =
            (static_cast<TagAction const &>(action).getActionType());

    struct Hash {
        std::size_t operator()(TagState const &state) const {
            return state.hash();
        }
    };
    typedef std::unordered_map<TagState, double, Hash> WeightMap;
    WeightMap weights;
    double weightTotal = 0;

    GridPosition newRobotPos(observation.getPosition());
    if (observation.seesOpponent()) {
        // If we saw the opponent, we must be in the same place.
        newParticles.push_back(
                std::make_unique<TagState>(newRobotPos, newRobotPos,
                        actionType == ActionType::TAG));
    } else {
        // We didn't see the opponent, so we must be in different places.
        for (solver::State const *state : previousParticles) {
            TagState const *tagState = static_cast<TagState const *>(state);
            GridPosition oldRobotPos(tagState->getRobotPosition());
            // Ignore states that do not match knowledge of the robot's position.
            if (newRobotPos != getMovedPos(oldRobotPos, actionType).first) {
                continue;
            }
            GridPosition oldOpponentPos(tagState->getOpponentPosition());
            std::vector<ActionType> actions(
                    makeOpponentActions(oldRobotPos, oldOpponentPos));
            std::vector<ActionType> newActions;
            for (ActionType opponentAction : actions) {
                if (getMovedPos(oldOpponentPos, opponentAction).first != newRobotPos) {
                    newActions.push_back(opponentAction);
                }
            }
            double probability = 1.0 / newActions.size();
            for (ActionType opponentAction : newActions) {
                GridPosition newOpponentPos = getMovedPos(oldOpponentPos, opponentAction).first;
                TagState newState(newRobotPos, newOpponentPos, false);
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
                newParticles.push_back(std::make_unique<TagState>(it->first));
            }
        }
    }
    return newParticles;
}

std::vector<std::unique_ptr<solver::State>> TagModel::generateParticles(
        solver::BeliefNode */*previousBelief*/, solver::Action const &action,
        solver::Observation const &obs, long nParticles) {
    std::vector<std::unique_ptr<solver::State>> newParticles;
    TagObservation const &observation =
            (static_cast<TagObservation const &>(obs));
    ActionType actionType =
            (static_cast<TagAction const &>(action).getActionType());
    GridPosition newRobotPos(observation.getPosition());
    if (observation.seesOpponent()) {
        // If we saw the opponent, we must be in the same place.
        while ((long)newParticles.size() < nParticles) {
            newParticles.push_back(
                std::make_unique<TagState>(newRobotPos, newRobotPos,
                        actionType == ActionType::TAG));
        }
    } else {
        while ((long)newParticles.size() < nParticles) {
            std::unique_ptr<solver::State> state = sampleStateUniform();
            solver::Model::StepResult result = generateStep(*state, action);
            if (obs == *result.observation) {
                newParticles.push_back(std::move(result.nextState));
            }
        }
    }
    return newParticles;
}


/* --------------- Pretty printing methods ----------------- */
void TagModel::dispCell(TagCellType cellType, std::ostream &os) {
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

void TagModel::drawEnv(std::ostream &os) {
    for (std::vector<TagCellType> &row : envMap_) {
        for (TagCellType cellType : row) {
            dispCell(cellType, os);
            os << " ";
        }
        os << endl;
    }
}

void TagModel::drawSimulationState(solver::BeliefNode const *belief,
        solver::State const &state, std::ostream &os) {
    TagState const &tagState = static_cast<TagState const &>(state);
    std::vector<solver::State const *> particles = belief->getStates();
    std::vector<std::vector<long>> particleCounts(nRows_,
            std::vector<long>(nCols_));
    for (solver::State const *particle : particles) {
        GridPosition opponentPos =
                static_cast<TagState const &>(*particle).getOpponentPosition();
        particleCounts[opponentPos.i][opponentPos.j] += 1;
    }

    std::vector<int> colors { 196, 161, 126, 91, 56, 21, 26, 31, 36, 41, 46 };
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
            bool hasRobot = (pos == tagState.getRobotPosition());
            bool hasOpponent = (pos == tagState.getOpponentPosition());
            if (hasRobot) {
                if (hasOpponent) {
                    os << "#";
                } else {
                    os << "r";
                }
            } else if (hasOpponent) {
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


/* ---------------------- Basic customizations  ---------------------- */
double TagModel::getDefaultHeuristicValue(solver::HistoryEntry const */*entry*/,
            solver::State const *state, solver::HistoricalData const */*data*/) {
    TagState const &tagState = static_cast<TagState const &>(*state);
    if (tagState.isTagged()) {
        return 0;
    }
    GridPosition robotPos = tagState.getRobotPosition();
    GridPosition opponentPos = tagState.getOpponentPosition();
    long dist = robotPos.manhattanDistanceTo(opponentPos);
    double nSteps = dist / opponentStayProbability_;
    double finalDiscount = std::pow(getDiscountFactor(), nSteps);
    double qVal = -moveCost_ * (1 - finalDiscount) / (1 - getDiscountFactor());
    qVal += finalDiscount * tagReward_;
    return qVal;
}


/* ------- Customization of more complex solver functionality  --------- */
std::vector<std::unique_ptr<solver::DiscretizedPoint>> TagModel::getAllActionsInOrder() {
    std::vector<std::unique_ptr<solver::DiscretizedPoint>> allActions;
    for (long code = 0; code < nActions_; code++) {
        allActions.push_back(std::make_unique<TagAction>(code));
    }
    return allActions;
}
std::unique_ptr<solver::ActionPool> TagModel::createActionPool(solver::Solver */*solver*/) {
    return std::make_unique<solver::EnumeratedActionPool>(this, getAllActionsInOrder());
}
std::unique_ptr<solver::Serializer> TagModel::createSerializer(solver::Solver *solver) {
    return std::make_unique<TagTextSerializer>(solver);
}
} /* namespace tag */
