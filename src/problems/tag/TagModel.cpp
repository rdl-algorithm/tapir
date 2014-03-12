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
#include "solver/changes/ChangeFlags.hpp"        // for ChangeFlags
#include "solver/indexing/FlaggingVisitor.hpp"
#include "solver/abstract-problem/Model.hpp"             // for Model::StepResult, Model
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/indexing/RTree.hpp"
#include "solver/indexing/SpatialIndexVisitor.hpp"             // for State, State::Hash, operator<<, operator==
#include "solver/abstract-problem/State.hpp"             // for State, State::Hash, operator<<, operator==
#include "solver/StatePool.hpp"

#include "solver/mappings/enumerated_actions.hpp"
#include "solver/mappings/discrete_observations_map.hpp"

#include "TagAction.hpp"
#include "TagObservation.hpp"
#include "TagState.hpp"                 // for TagState

using std::cout;
using std::endl;
namespace po = boost::program_options;

namespace tag {
TagModel::TagModel(RandomGenerator *randGen, po::variables_map vm) :
    ModelWithProgramOptions(randGen, vm),
    ModelWithEnumeratedActions(getAllActionsInOrder()),
    moveCost_(vm["problem.moveCost"].as<double>()),
    tagReward_(vm["problem.tagReward"].as<double>()),
    failedTagPenalty_(vm["problem.failedTagPenalty"].as<double>()),
    opponentStayProbability_(
            vm["problem.opponentStayProbability"].as<double>()),
    nRows_(0), // to be updated
    nCols_(0), // to be updated
    mapText_(), // will be pushed to
    envMap_(), // will be pushed to
    changes_(),
    nActions_(5),
    nStVars_(5),
    minVal_(-failedTagPenalty_ / (1 - getDiscountFactor())),
    maxVal_(tagReward_)
    {
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
    cout << "Constructed the TagModel" << endl;
    cout << "Discount: " << getDiscountFactor() << endl;
    cout << "Size: " << nRows_ << " by " << nCols_ << endl;
    cout << "move cost: " << moveCost_ << endl;
    cout << "nActions: " << nActions_ << endl;
    cout << "nStVars: " << nStVars_ << endl;
    cout << "Example States: " << endl;
    for (int i = 0; i < 5; i++) {
        std::unique_ptr<solver::State> state = sampleAnInitState();
        cout << *state << " Heuristic: " << getHeuristicValue(*state) << endl;
    }
    cout << "nParticles: " << getNParticles() << endl;
    cout << "Environment:" << endl << endl;
    drawEnv(cout);
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

double TagModel::getHeuristicValue(solver::State const &state) {
    TagState const &tagState = static_cast<TagState const &>(state);
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

double TagModel::getDefaultVal() {
    return minVal_;
}

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
    GridPosition newRobotPos = getMovedPos(robotPos, tagAction.getActionType());
    if (!isValid(newRobotPos)) {
        return std::make_pair(
                std::make_unique<TagState>(robotPos, newOpponentPos, false),
                false);
    }
    return std::make_pair(std::make_unique<TagState>(
                    newRobotPos, newOpponentPos, false), true);
}

std::vector<ActionType> TagModel::makeOpponentActions(
        GridPosition const &robotPos,
        GridPosition const &opponentPos) {
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
    ActionType action = actions[std::uniform_int_distribution<long>(
                    0, actions.size() - 1)(*getRandomGenerator())];
    GridPosition newOpponentPos = getMovedPos(opponentPos, action);
    if (!isValid(newOpponentPos)) {
        newOpponentPos = opponentPos;
    }
    return newOpponentPos;
}

GridPosition TagModel::getMovedPos(GridPosition const &position,
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
        message << "Invalid action: " << (long)action;
        debug::show_message(message.str());
        break;
    }
    return movedPos;
}

bool TagModel::isValid(GridPosition const &position) {
    return (position.i >= 0 && position.i < nRows_ && position.j >= 0
            && position.j < nCols_ && envMap_[position.i][position.j] != WALL);
}

std::unique_ptr<solver::Observation> TagModel::makeObservation(
        solver::Action const & /*action*/,
        TagState const &nextState) {
    return std::make_unique<TagObservation>(nextState.getRobotPosition(),
            nextState.getRobotPosition() == nextState.getOpponentPosition());
}

double TagModel::generateReward(
        solver::State const &state,
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
        solver::State const &state,
        solver::Action const &action,
        solver::TransitionParameters const */*tp*/) {
    return makeNextState(static_cast<TagState const &>(state), action).first;
}

std::unique_ptr<solver::Observation> TagModel::generateObservation(
        solver::State const */*state*/,
        solver::Action const &action,
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

std::vector<std::unique_ptr<solver::State>> TagModel::generateParticles(
        solver::BeliefNode */*previousBelief*/,
        solver::Action const &action, solver::Observation const &obs,
        std::vector<solver::State const *> const &previousParticles) {
    std::vector<std::unique_ptr<solver::State>> newParticles;
    TagObservation const &observation = (
            static_cast<TagObservation const &>(obs));
    ActionType actionType = (
            static_cast<TagAction const &>(action).getActionType());

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
        newParticles.push_back(std::make_unique<TagState>(newRobotPos,
                        newRobotPos, actionType == ActionType::TAG));
    } else {
        // We didn't see the opponent, so we must be in different places.
        for (solver::State const *state : previousParticles) {
            TagState const *tagState = static_cast<TagState const *>(state);
            GridPosition oldRobotPos(tagState->getRobotPosition());
            // Ignore states that do not match knowledge of the robot's position.
            if (newRobotPos != getMovedPos(oldRobotPos, actionType)) {
                continue;
            }
            GridPosition oldOpponentPos(tagState->getOpponentPosition());
            std::vector<ActionType> actions(makeOpponentActions(oldRobotPos,
                            oldOpponentPos));
            std::vector<ActionType> newActions;
            for (ActionType opponentAction : actions) {
                if (getMovedPos(oldOpponentPos,
                        opponentAction) != newRobotPos) {
                    newActions.push_back(opponentAction);
                }
            }
            double probability = 1.0 / newActions.size();
            for (ActionType opponentAction : newActions) {
                GridPosition newOpponentPos = getMovedPos(oldOpponentPos,
                        opponentAction);
                TagState newState(newRobotPos, newOpponentPos, false);
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
                newParticles.push_back(std::make_unique<TagState>(it->first));
            }
        }
    }
    return newParticles;
}

std::vector<std::unique_ptr<solver::State>> TagModel::generateParticles(
        solver::BeliefNode */*previousBelief*/,
        solver::Action const &action, solver::Observation const &obs) {
    std::vector<std::unique_ptr<solver::State>> newParticles;
    TagObservation const &observation = (
            static_cast<TagObservation const &>(obs));
    ActionType actionType = (
            static_cast<TagAction const &>(action).getActionType());
    GridPosition newRobotPos(observation.getPosition());
    if (observation.seesOpponent()) {
        // If we saw the opponent, we must be in the same place.
        newParticles.push_back(
                std::make_unique<TagState>(newRobotPos, newRobotPos,
                        actionType == ActionType::TAG));
    } else {
        while (newParticles.size() < getNParticles()) {
            std::unique_ptr<solver::State> state = sampleStateUniform();
            solver::Model::StepResult result = generateStep(*state, action);
            if (obs == *result.observation) {
                newParticles.push_back(std::move(result.nextState));
            }
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

std::vector<long> TagModel::loadChanges(char const *changeFilename) {
    std::vector<long> changeTimes;
    std::ifstream ifs;
    ifs.open(changeFilename);
    std::string line;
    while (std::getline(ifs, line)) {
        std::string tmpStr;
        long time;
        long nChanges;
        std::istringstream(line) >> tmpStr >> time >> tmpStr >> nChanges;

        changes_[time] = std::vector<TagChange>();
        changeTimes.push_back(time);
        for (int i = 0; i < nChanges; i++) {
            std::getline(ifs, line);
            TagChange change;
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

void TagModel::update(long time, solver::StatePool *pool) {
    for (TagChange &change : changes_[time]) {
        cout << change.changeType << " " << change.i0 << " " << change.j0 << " "
                << change.i1 << " " << change.j1 << endl;
        if (change.changeType == "Add Obstacles") {
            for (long i = static_cast<long>(change.i0); i <= change.i1; i++) {
                for (long j = static_cast<long>(change.j0); j <= change.j1; j++) {
                    envMap_[i][j] = TagCellType::WALL;
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
                    envMap_[i][j] = TagCellType::EMPTY;
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

void TagModel::drawSimulationState(
        std::vector<solver::State const *> particles,
        solver::State const &state,
        std::ostream &os) {
    TagState const &tagState = static_cast<TagState const &>(state);
    os << "Belief has " << particles.size() << " particles." << endl;
    os << state << endl;
    std::vector<std::vector<long>> particleCounts(nRows_,
            std::vector<long>(nCols_));
    for (solver::State const *particle : particles) {
        GridPosition opponentPos = static_cast<TagState const &>(
                *particle).getOpponentPosition();
        particleCounts[opponentPos.i][opponentPos.j] += 1;
    }

    for (std::size_t i = 0; i < envMap_.size(); i++) {
        for (std::size_t j = 0; j < envMap_[0].size(); j++) {
            double proportion = (double) particleCounts[i][j]
                    / particles.size();
            if (hasColorOutput()) {
                std::vector<int> colors {196,
                    161, 126, 91, 56, 21,
                    26, 31, 36, 41, 46};
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

std::vector<std::unique_ptr<solver::DiscretizedPoint>>
TagModel::getAllActionsInOrder() {
    std::vector<std::unique_ptr<solver::DiscretizedPoint>> allActions;
    for (long code = 0; code < 5; code++) {
        allActions.push_back(std::make_unique<TagAction>(code));
    }
    return allActions;
}
} /* namespace tag */
