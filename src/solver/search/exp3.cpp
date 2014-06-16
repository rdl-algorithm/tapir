#include "exp3.hpp"

#include "solver/abstract-problem/Model.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/Solver.hpp"

namespace solver {
MultipleStrategiesExp3::MultipleStrategiesExp3(Solver *solver,
        double strategyExplorationCoefficient,
        std::vector<std::unique_ptr<SearchStrategy>> strategies) :
            solver_(solver),
            model_(solver_->getModel()),
            strategyExplorationCoefficient_(strategyExplorationCoefficient),
            strategies_() {
    for (unsigned long index = 0; index < strategies.size(); index++) {
        StrategyInfo info;
        info.strategyNo = index;
        info.strategy = std::move(strategies[index]);
        info.weight = 1.0 / strategies.size();
        info.probability = 1.0 / strategies.size();
        strategies_.push_back(std::move(info));
    }
}

StrategyInfo *MultipleStrategiesExp3::sampleAStrategy(
        std::unordered_set<long> strategiesToExclude) {
    std::vector<double> weights;
    for (StrategyInfo &info : strategies_) {
        if (strategiesToExclude.count(info.strategyNo) == 0) {
            weights.push_back(info.probability);
        }
    }
    if (weights.empty()) {
        return nullptr;
    }
    std::discrete_distribution<long> distribution(weights.begin(), weights.end());
    long strategyNo = distribution(*model_->getRandomGenerator());
    return &strategies_[strategyNo];
}

void MultipleStrategiesExp3::updateStrategyWeights(long strategyNo, double timeUsed,
        double deltaQ) {
    StrategyInfo &strategyInfo = strategies_[strategyNo];
    if (deltaQ < 0.0) {
        deltaQ = 0.0;
    }
    strategyInfo.timeSpent += timeUsed;
    strategyInfo.numberOfTimesUsed++;
    strategyInfo.weight *= std::exp(
            strategyExplorationCoefficient_ * (deltaQ / model_->getMaxVal())
                    / (2 * strategyInfo.probability));

    double weightTotal = 0.0;
    for (StrategyInfo &info : strategies_) {
        weightTotal += info.weight;
    }
    double probabilityTotal = 0.0;
    for (StrategyInfo &info : strategies_) {
        info.probability = ((1 - strategyExplorationCoefficient_) * info.weight / weightTotal
                + strategyExplorationCoefficient_ / 2);
        info.probability *= (info.numberOfTimesUsed + 1) / (info.timeSpent + 1);
        probabilityTotal += info.probability;
    }
    for (StrategyInfo &info : strategies_) {
        info.probability /= probabilityTotal;
    }
}

SearchStatus MultipleStrategiesExp3::extendSequence(HistorySequence *sequence, long maximumDepth,
        bool doBackup) {
    BeliefNode *rootNode = sequence->getFirstEntry()->getAssociatedBeliefNode();
    double initialRootQValue = rootNode->getQValue();

    std::unordered_set<long> failedStrategies;
    while (true) {
        StrategyInfo *info = sampleAStrategy(failedStrategies);
        // No more strategies - give up.
        if (info == nullptr) {
            return SearchStatus::UNINITIALIZED;
        }

        double startTime = abt::clock_ms();
        SearchStatus status = info->strategy->extendSequence(sequence, maximumDepth, doBackup);
        double timeUsed = abt::clock_ms() - startTime;

        // If the strategy initialized, we backup, update weights, and we're done.
        if (status != SearchStatus::UNINITIALIZED) {
            // We can update the weights for Exp3, but only if the sequence was backed up.
            if (doBackup) {
                double newRootQValue = rootNode->getQValue();
                updateStrategyWeights(info->strategyNo, timeUsed, newRootQValue - initialRootQValue);
            }
            return status;
        }

        // The strategy failed to initialize - update the weights, but keep trying other strategies.
        updateStrategyWeights(info->strategyNo, timeUsed, 0.0);
        failedStrategies.insert(info->strategyNo);
    }
    return SearchStatus::UNINITIALIZED;
}
} /* namespace solver */
