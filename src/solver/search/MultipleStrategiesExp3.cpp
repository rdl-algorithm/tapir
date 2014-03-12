#include "MultipleStrategiesExp3.hpp"

#include "solver/abstract-problem/Model.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/HistoryEntry.hpp"

namespace solver {
MultipleStrategiesExp3::MultipleStrategiesExp3(
        std::vector<std::unique_ptr<SearchStrategy>> strategies,
        Model *model, double strategyExplorationCoefficient) :
                strategies_(),
                model_(model),
                strategyExplorationCoefficient_(strategyExplorationCoefficient) {
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
                SearchInstance(solver, sequence, maximumDepth),
                parent_(parent),
                failedStrategies_(),
                currentStrategyNo_(0),
                currentInstance_(nullptr),
                currentInstanceStartTime_(0),
                initialRootQValue_(0) {
}

SearchStatus MultipleStrategiesExp3Instance::initialize() {
    initialRootQValue_ = sequence_->getFirstEntry()->getAssociatedBeliefNode()->getQValue();
    status_ = SearchStatus::UNINITIALIZED;
    while (true) {
        StrategyInfo *info = parent_->sampleAStrategy(failedStrategies_);
        if (info == nullptr) {
            // We are out of strategies, so we give up.
            break;
        }
        currentStrategyNo_ = info->strategyNo;
        currentInstance_ = info->strategy->createSearchInstance(solver_, sequence_, maximumDepth_);
        currentInstanceStartTime_ = std::clock();
        status_ = currentInstance_->initialize();
        if (status_ == SearchStatus::INITIAL) {
            // This strategy seems OK, so we'll try it.
            break;
        }
        // The strategy failed to initialize; we should record the failed attempt.
        failedStrategies_.insert(currentStrategyNo_);
        double timeTaken = (std::clock() - currentInstanceStartTime_) * 1000.0 / CLOCKS_PER_SEC;
        parent_->updateStrategyWeights(currentStrategyNo_, timeTaken, 0.0);
    }
    return status_;
}

std::pair<SearchStatus, std::unique_ptr<Action>>
MultipleStrategiesExp3Instance::getStatusAndNextAction() {
    return currentInstance_->getStatusAndNextAction();
}

SearchStatus MultipleStrategiesExp3Instance::finalize() {
    double timeUsed = (std::clock() - currentInstanceStartTime_) * 1000.0 / CLOCKS_PER_SEC;
    double newRootQValue = sequence_->getEntry(0)->getAssociatedBeliefNode()->getQValue();
    double deltaQ = newRootQValue - initialRootQValue_;
    parent_->updateStrategyWeights(currentStrategyNo_, timeUsed, deltaQ);
    return status_;
}
} /* namespace solver */
