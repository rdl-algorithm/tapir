#include "RockSampleMdpSolver.hpp"

#include <iostream>

#include "problems/shared/GridPosition.hpp"

#include "solver/HistoryEntry.hpp"

#include "RockSampleModel.hpp"
#include "RockSampleState.hpp"

namespace rocksample {
RockSampleMdpSolver::RockSampleMdpSolver(RockSampleModel *model) :
            model_(model),
            valueMap_() {
}

void RockSampleMdpSolver::solve() {
    if (model_->options_->hasVerboseOutput) {
        std::cout << "Solving MDP...";
        std::cout.flush();
    }

    valueMap_.clear();

    // States are represented as pairs of integers.
    // The first number encodes the states of the rocks.
    // The second number encodes the current position (which rock you're on top of).
    std::set<std::pair<int, int>> states;
    std::set<std::pair<int, int>> newStates;

    // Start with a base case where all rocks are bad.
    for (int i = 0; i < model_->nRocks_; i++) {
        newStates.insert(std::make_pair(0, i));
    }

    for (int i = 0; i < model_->nRocks_; i++) {
        states = newStates;
        newStates.clear();
        for (std::pair<int, int> entry : states) {
            long rockStateCode = entry.first;
            long positionNo = entry.second;
            GridPosition pos = model_->rockPositions_[positionNo];

            // The value of going straight to the goal is a simple lower bound.
            double value = calculateQValue(pos, rockStateCode, -1);
            if (valueMap_[entry] < value) {
                valueMap_[entry] = value;
            } else {
                value = valueMap_[entry];
            }

            for (int j = 0; j < model_->nRocks_; j++) {
                // Try propagating to the rock we came from - it can't  be
                // the current rock, and we must have sampled it so it
                // must be bad now.
                if (j != positionNo && (rockStateCode & (1 << j)) == 0) {
                    long prevCode = rockStateCode | (1 << positionNo);
                    GridPosition prevPos = model_->rockPositions_[j];
                    std::pair<int, int> index = std::make_pair(prevCode, j);
                    double prevValue = calculateQValue(prevPos, prevCode, positionNo);
                    if (valueMap_[index] < prevValue) {
                        valueMap_[index] = prevValue;
                    }
                    newStates.insert(index);
                }
            }
        }
    }

    if (model_->options_->hasVerboseOutput) {
        std::cout << "                   Done." << std::endl << std::endl;
    }
}

double RockSampleMdpSolver::getQValue(RockSampleState const &state) const {
    GridPosition pos = state.getPosition();
    long rockStateCode = model_->encodeRocks(state.getRockStates());

    // Check the value of leaving the map.
    double value = calculateQValue(pos, rockStateCode, -1);

    for (int i = 0; i < model_->nRocks_; i++) {
        // Only try to sample good rocks!
        if ((rockStateCode & (1 << i)) != 0) {
            double newValue = calculateQValue(pos, rockStateCode, i);
            if (newValue > value) {
                value = newValue;
            }
        }
    }
    return value;
}

double RockSampleMdpSolver::calculateQValue(GridPosition pos, long rockStateCode,
        long action) const {
    long actionsUntilReward = model_->getDistance(pos, action);
    if (actionsUntilReward == -1) {
        // Impossible!
        return -std::numeric_limits<double>::infinity();
    }

    double reward;
    double nextQValue;

    if (action == -1) {
        // Reward is received as you move into the goal square, not afterwards.
        actionsUntilReward -= 1;
        reward = model_->exitReward_;
        nextQValue = 0; // Terminal.
    } else {
        if ((rockStateCode & (1 << action)) == 0) {
            debug::show_message("ERROR: No reward for this action!");
            reward = -model_->badRockPenalty_;
        } else {
            reward = model_->goodRockReward_;
        }

        long nextCode = rockStateCode & ~(1 << action);
        nextQValue = valueMap_.at(std::make_pair(nextCode, action));
    }

    double discountFactor = model_->options_->discountFactor;
    return std::pow(discountFactor, actionsUntilReward) * (reward + discountFactor * nextQValue);
}


RockSampleMdpParser::RockSampleMdpParser(RockSampleModel *model) :
        model_(model) {
}

solver::Heuristic RockSampleMdpParser::parse(solver::Solver */*solver*/,
        std::vector<std::string> /*args*/) {
    if (model_->getMdpSolver() == nullptr) {
        model_->makeMdpSolver();
    }
    return [this] (solver::HistoryEntry const *, solver::State const *state,
            solver::HistoricalData const *) {
        RockSampleMdpSolver *solver = model_->getMdpSolver();
        return solver->getQValue(static_cast<RockSampleState const &>(*state));
    };
}
} /* namespace rocksample */
