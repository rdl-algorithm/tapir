#include "RockSampleMdpSolver.hpp"

#include <iostream>

#include "problems/shared/GridPosition.hpp"

#include "RockSampleModel.hpp"
#include "RockSampleState.hpp"

namespace rocksample {
    RockSampleMdpSolver::RockSampleMdpSolver(RockSampleModel *model) :
            model_(model),
            valueMap_() {
    }

    void RockSampleMdpSolver::solve() {
        std::set<std::pair<int, int>> entries;
        std::set<std::pair<int, int>> newEntries;
        for (int i = 0; i < model_->nRocks_; i++) {
            newEntries.insert(std::make_pair(0, i));
        }
        for (int i = 0; i < model_->nRocks_; i++) {
            entries = newEntries;
            newEntries.clear();
            for (std::pair<int, int> entry : entries) {
                long rockStateCode = entry.first;
                long positionNo = entry.second;
                GridPosition pos = model_->rockPositions_[positionNo];

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
                        newEntries.insert(index);
                    }
                }
            }
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

    double RockSampleMdpSolver::calculateQValue(GridPosition pos,
            long rockStateCode, long action) const {
        long actionsUntilReward;
        double reward;
        double nextQValue;

        if (action == -1) {
            actionsUntilReward = model_->nCols_ - 2 - pos.j;
            reward = model_->exitReward_;
            nextQValue = 0; // Terminal.
        } else {
            GridPosition nextPos = model_->rockPositions_[action];
            actionsUntilReward = pos.manhattanDistanceTo(nextPos);

            if ((rockStateCode & (1 << action)) == 0) {
                debug::show_message("ERROR: No reward for this action!");
                reward = -model_->badRockPenalty_;
            } else {
                reward = model_->goodRockReward_;
            }

            long nextCode = rockStateCode & ~(1 << action);
            nextQValue = valueMap_.at(std::make_pair(nextCode, action));
        }

        double discountFactor = model_->getDiscountFactor();
        return std::pow(discountFactor, actionsUntilReward) * (
                reward + discountFactor * nextQValue);
    }

} /* namespace rocksample */
