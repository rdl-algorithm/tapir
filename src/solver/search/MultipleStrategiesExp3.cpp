#include "MultipleStrategiesExp3.hpp"

namespace solver {
MultipleStrategiesExp3::MultipleStrategiesExp3(
        std::vector<std::unique_ptr<SearchStrategy>> strategies,
        double strategyExplorationCoefficient) :
                strategies_(),
                strategyExplorationCoefficient_(strategyExplorationCoefficient) {
    for (std::unique_ptr<SearchStrategy> &strategy : strategies) {
        strategies_.push_back(StrategyInfo());
        strategies_.rbegin()->strategy = std::move(strategy);
    }
}

std::unique_ptr<SearchInstance> MultipleStrategiesExp3::createSearchInstance(
       Solver *solver, HistorySequence *sequence) {
    long strategyNo = 0;
    // TODO Select a strategy probabilistically using Exp3
    std::unique_ptr<SearchInstance> searchInstance = (
            strategies_[strategyNo].strategy->createSearchInstance(solver, sequence));
    return std::make_unique<MultipleStrategiesExp3Instance>(this, 0,
            std::move(searchInstance));
}

MultipleStrategiesExp3Instance::MultipleStrategiesExp3Instance(
        MultipleStrategiesExp3 *parent, long strategyNo,
        std::unique_ptr<SearchInstance> searchInstance) :
                parent_(parent),
                strategyNo_(strategyNo),
                searchInstance_(std::move(searchInstance)) {
}

virtual std::pair<SearchStatus, std::unique_ptr<Action>>
MultipleStrategiesExp3Instance::getStatusAndNextAction() {
    return searchInstance_->getStatusAndNextAction();
}
} /* namespace solver */
