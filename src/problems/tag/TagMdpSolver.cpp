#include "TagMdpSolver.hpp"

#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

#include "global.hpp"

#include "problems/shared/parsers.hpp"
#include "problems/shared/policy_iteration.hpp"
#include "solver/abstract-problem/heuristics/Heuristic.hpp"

#include "TagModel.hpp"
#include "TagState.hpp"

namespace tag {
/* ---------------------- TagMdpSolver --------------------- */
TagMdpSolver::TagMdpSolver(TagModel *model) :
            model_(model),
            valueMap_() {
}

void TagMdpSolver::solve() {
    if (model_->hasVerboseOutput()) {
        std::cout << "Solving MDP...";
        std::cout.flush();
    }

    valueMap_.clear();

    // Enumerated vector of actions.
    std::vector<std::unique_ptr<solver::DiscretizedPoint>> allActions = (
            model_->getAllActionsInOrder());

    // Vector of valid grid positions.
    std::vector<GridPosition> emptyCells;
    for (long row = 0; row < model_->getNRows(); row++) {
        for (long col = 0; col < model_->getNCols(); col++) {
            // Ignore impossible states.
            if (model_->envMap_[row][col] == TagModel::TagCellType::EMPTY) {
                emptyCells.emplace_back(row, col);
            }
        }
    }

    // An initial policy
    mdp::Policy policy;
    // Enumerated vector of all states.
    std::vector<TagState> allStates;
    // Mapping from each state to its assigned index.
    std::unordered_map<TagState, int> stateIndex;

    int index = 0;
    for (GridPosition const &robotPos : emptyCells) {
        for (GridPosition const &opponentPos : emptyCells) {
            TagState state(robotPos, opponentPos, false);

            ActionType initialAction;
            // Set a default initial action for our initial policy.
            if (robotPos == opponentPos) {
                // Same posn => tag
                initialAction = ActionType::TAG;
            } else {
                // Different posn => try to move towards the opponent.
                long deltaI = opponentPos.i - robotPos.i;
                long deltaJ = opponentPos.j - robotPos.j;

                // Try to move along the axis where we have the greatest difference.
                if (std::abs(deltaI > std::abs(deltaJ))) {
                    if (deltaI > 0) {
                        initialAction = ActionType::SOUTH;
                    } else {
                        initialAction = ActionType::NORTH;
                    }
                } else {
                    if (deltaJ > 0) {
                        initialAction = ActionType::EAST;
                    } else {
                        initialAction = ActionType::WEST;
                    }
                }
            }

            allStates.push_back(state);
            stateIndex[state] = index;
            policy.push_back(static_cast<int>(initialAction));
            index += 1;
        }
    }
    // An index of size (past the end of the array!!) will be used to represent terminal states.
    // A default action for the terminal state is meaningless, but we need it anyway.
    policy[index] = static_cast<int>(ActionType::TAG);


    // Initialise the state transitions.
    std::vector<std::vector<std::unordered_map<int, std::pair<double, double>>>> transitions;
    transitions.resize(allStates.size() + 1);
    for (unsigned int stateNo = 0; stateNo <= allStates.size(); stateNo++) {
        TagState const &state = allStates[stateNo];
        GridPosition robotPos = state.getRobotPosition();
        GridPosition opponentPos = state.getOpponentPosition();

        std::vector<ActionType> opponentActions = (
                model_->makeOpponentActions(robotPos, opponentPos));

        // Generate a distribution of possible opponent positions.
        std::unordered_map<GridPosition, double> nextOpponentPosDistribution;
        for (ActionType opponentAction : opponentActions) {
            GridPosition nextOpponentPos = model_->getMovedPos(opponentPos, opponentAction).first;
            nextOpponentPosDistribution[nextOpponentPos] += 1.0;
        }
        for (auto &entry : nextOpponentPosDistribution) {
            entry.second /= opponentActions.size();
        }

        transitions[stateNo].resize(allActions.size());
        for (unsigned int actionNo = 0; actionNo < allActions.size(); actionNo++) {
            ActionType actionType = static_cast<ActionType>(actionNo);
            auto &nextStateTransitions = transitions[stateNo][actionNo];

            double reward = -1;
            // The TAG action needs special handling.
            if (actionType == ActionType::TAG) {
                if (robotPos == opponentPos) {
                    // Transition to the terminal state
                    nextStateTransitions[allStates.size()] = std::make_pair(1.0, 10.0);
                    continue;
                } else {
                    // Failure to tag => proceed as usual.
                    reward = -10;
                }
            }
            GridPosition nextRobotPos = model_->getMovedPos(robotPos, actionType).first;
            for (auto &entry : nextOpponentPosDistribution) {
                TagState nextState(nextRobotPos, entry.first, false);
                int nextStateIndex = stateIndex[nextState];
                nextStateTransitions[nextStateIndex] = std::make_pair(entry.second, reward);
            }
        }
    }

    std::function<std::vector<int>(int, int)> possibleNextStates =
            [transitions](int state, int action) {
                std::vector<int> nextStates;
                for (auto const &entry : transitions[state][action]) {
                    nextStates.push_back(entry.first);
                }
                return std::move(nextStates);
            };
    std::function<double(int, int, int)> transitionProbability =
            [transitions](int state, int action, int nextState) {
                return transitions[state][action].at(nextState).first;
            };
    std::function<double(int, int, int)> reward =
            [transitions](int state, int action, int nextState) {
                return transitions[state][action].at(nextState).second;
            };

    mdp::solve(policy, model_->getDiscountFactor(),
            allStates.size() + 1, allActions.size(),
            possibleNextStates, transitionProbability, reward);

    std::vector<double> stateValues;
    stateValues[-1] =

    // Now put all of the state values into our map.
    for (unsigned int stateNo = 0; stateNo < allStates.size(); stateNo++) {
        valueMap_[allStates[stateNo]] = stateValues[stateNo];
    }

    if (model_->hasVerboseOutput()) {
        std::cout << "                   Done." << std::endl << std::endl;
    }
}

double TagMdpSolver::getQValue(TagState const &state) const {
    if (state.isTagged()) {
        return 0; // Terminal; the reward is applied on the previous timestep.
    }

    try {
        return valueMap_.at(state);
    } catch (std::out_of_range const &oor) {
        // INVALID STATE => return 0

        // std::cout << "NOTE: Queried heuristic value of an invalid state." << std::endl;
        return 0;
    }
}


/* ---------------------- TagMdpParser --------------------- */
TagMdpParser::TagMdpParser(TagModel *model) :
        model_(model) {
}

solver::Heuristic TagMdpParser::parse(solver::Solver */*solver*/,
        std::vector<std::string> /*args*/) {
    if (model_->getMdpSolver() == nullptr) {
        model_->makeMdpSolver();
    }
    return [this] (solver::HistoryEntry const *, solver::State const *state,
            solver::HistoricalData const *) {
        TagMdpSolver *solver = model_->getMdpSolver();
        return solver->getQValue(static_cast<TagState const &>(*state));
    };
}
} /* namespace tag */
