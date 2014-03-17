#include "MultipleStrategiesExp3.hpp"

#include "solver/abstract-problem/Model.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/HistoryEntry.hpp"

namespace solver {
MultipleStrategiesExp3::MultipleStrategiesExp3(double strategyExplorationCoefficient,
        std::vector<std::unique_ptr<SearchStrategy>> strategies, Model *model) :
            strategyExplorationCoefficient_(strategyExplorationCoefficient),
            strategies_(),
            model_(model) {
    for (unsigned long index = 0; index < strategies.size(); index++) {
        StrategyInfo info;
        info.strategyNo = index;
        info.strategy = std::move(strategies[index]);
        info.weight = 1.0 / strategies.size();
        info.probability = 1.0 / strategies.size();
        strategies_.push_back(std::move(info));
    }
}

std::unique_ptr<SearchInstance> MultipleStrategiesExp3::createSearchInstance(
       Solver *solver, HistorySequence *sequence, long maximumDepth) {
    return std::make_unique<MultipleStrategiesExp3Instance>(this,
            solver, sequence, maximumDepth);
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

void MultipleStrategiesExp3::updateStrategyWeights(long strategyNo,
        double timeUsed, double deltaQ) {
    StrategyInfo &strategyInfo = strategies_[strategyNo];
    if (deltaQ < 0.0) {
        deltaQ = 0.0;
    }
    strategyInfo.timeSpent += timeUsed;
    strategyInfo.numberOfTimesUsed++;
    strategyInfo.weight *= std::exp(strategyExplorationCoefficient_
            * (deltaQ / model_->getMaxVal()) / (2 * strategyInfo.probability));

    double weightTotal = 0.0;
    for (StrategyInfo &info : strategies_) {
        weightTotal += info.weight;
    }
    double probabilityTotal = 0.0;
    for (StrategyInfo &info : strategies_) {
        info.probability = ((1 - strategyExplorationCoefficient_) * info.weight
                / weightTotal + strategyExplorationCoefficient_ / 2);
        info.probability *= (info.numberOfTimesUsed + 1) / (info.timeSpent + 1);
        probabilityTotal += info.probability;
    }
    for (StrategyInfo &info : strategies_) {
        info.probability /= probabilityTotal;
    }
}

MultipleStrategiesExp3Instance::MultipleStrategiesExp3Instance(
        MultipleStrategiesExp3 *parent,
        Solver *solver, HistorySequence *sequence, long maximumDepth) :
                parent_(parent),
                solver_(solver),
                sequence_(sequence),
                maximumDepth_(maximumDepth),
                failedStrategies_(),
                currentStrategyNo_(0),
                currentInstance_(nullptr),
                currentInstanceStartTime_(0),
                initialRootQValue_(0) {
}

SearchStatus MultipleStrategiesExp3Instance::initialize() {
    initialRootQValue_ = sequence_->getFirstEntry()->getAssociatedBeliefNode()->getQValue();
    while (true) {
        StrategyInfo *info = parent_->sampleAStrategy(failedStrategies_);
        if (info == nullptr) {
            // We are out of strategies, so we give up.
            return  SearchStatus::UNINITIALIZED;;
        }
        currentStrategyNo_ = info->strategyNo;
        currentInstance_ = info->strategy->createSearchInstance(solver_, sequence_, maximumDepth_);
        currentInstanceStartTime_ = std::clock();
        if (currentInstance_->initialize() == SearchStatus::INITIAL) {
            // This strategy seems OK, so we'll try it.
            return SearchStatus::INITIAL;
        }
        // The strategy failed to initialize; we should record the failed attempt.
        failedStrategies_.insert(currentStrategyNo_);
        double timeTaken = (std::clock() - currentInstanceStartTime_) * 1000.0 / CLOCKS_PER_SEC;
        parent_->updateStrategyWeights(currentStrategyNo_, timeTaken, 0.0);
    }
    return SearchStatus::UNINITIALIZED;
}

SearchStatus MultipleStrategiesExp3Instance::extendSequence() {
    return currentInstance_->extendSequence();
}

SearchStatus MultipleStrategiesExp3Instance::finalize() {
    double timeUsed = (std::clock() - currentInstanceStartTime_) * 1000.0 / CLOCKS_PER_SEC;
    double newRootQValue = sequence_->getFirstEntry()->getAssociatedBeliefNode()->getQValue();
    double deltaQ = newRootQValue - initialRootQValue_;
    parent_->updateStrategyWeights(currentStrategyNo_, timeUsed, deltaQ);
    return currentInstance_->finalize();
}
} /* namespace solver */
