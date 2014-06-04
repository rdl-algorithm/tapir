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

std::unique_ptr<SearchInstance> MultipleStrategiesExp3::createSearchInstance(SearchStatus &status,
        HistorySequence *sequence, long maximumDepth) {
    return std::make_unique<MultipleStrategiesExp3Instance>(status, sequence, maximumDepth, solver_,
            this);
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

MultipleStrategiesExp3Instance::MultipleStrategiesExp3Instance(SearchStatus &status,
        HistorySequence *sequence, long maximumDepth, Solver *solver,
        MultipleStrategiesExp3 *parent) :
            SearchInstance(status),
            sequence_(sequence),
            maximumDepth_(maximumDepth),
            solver_(solver),
            parent_(parent),
            strategyNo_(-1),
            currentInstance_(nullptr),
            timeUsed_(0) {
    std::unordered_set<long> failedStrategies;
    while (true) {
        StrategyInfo *info = parent_->sampleAStrategy(failedStrategies);
        if (info == nullptr) {
            return; // We are out of strategies, so we give up.
        }
        strategyNo_ = info->strategyNo;
        double startTime = abt::clock_ms();
        currentInstance_ = info->strategy->createSearchInstance(status_, sequence_, maximumDepth_);
        timeUsed_ = abt::clock_ms() - startTime;
        // If initialization was successful, break out of the loop.
        if (status_ == SearchStatus::INITIAL) {
            break;
        }

        // The strategy failed to initialize; we should record the failed attempt.
        failedStrategies.insert(strategyNo_);
        parent_->updateStrategyWeights(strategyNo_, timeUsed_, 0.0);
    }
}

void MultipleStrategiesExp3Instance::extendSequence() {
    double initialRootQValue = sequence_->getFirstEntry()->getAssociatedBeliefNode()->getQValue();
    double startTime = abt::clock_ms();
    currentInstance_->extendSequence();
    timeUsed_ += (abt::clock_ms() - startTime);
    solver_->doBackup(); // Do a backup so the root Q value gets updated.
    double newRootQValue = sequence_->getFirstEntry()->getAssociatedBeliefNode()->getQValue();
    double deltaQ = newRootQValue - initialRootQValue;
    parent_->updateStrategyWeights(strategyNo_, timeUsed_, deltaQ);
}
} /* namespace solver */
