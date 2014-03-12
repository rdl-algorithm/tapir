#ifndef SOLVER_MULTIPLESTRATEGIESEXP3_HPP_
#define SOLVER_MULTIPLESTRATEGIESEXP3_HPP_

#include <ctime>

#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

#include "SearchStatus.hpp"
#include "SearchStrategy.hpp"

namespace solver {
struct StrategyInfo {
    long strategyNo = 0;
    std::unique_ptr<SearchStrategy> strategy = nullptr;
    double timeSpent = 0.0;
    double weight = 0.0;
    double probability = 0.0;
    long numberOfTimesUsed = 0;
};

class MultipleStrategiesExp3: public SearchStrategy {
    friend class MultipleStrategiesExp3Instance;
  public:
    MultipleStrategiesExp3(std::vector<std::unique_ptr<SearchStrategy>> strategies,
            Model *model, double strategyExplorationCoefficient);
    virtual ~MultipleStrategiesExp3() = default;
    _NO_COPY_OR_MOVE(MultipleStrategiesExp3);

    virtual std::unique_ptr<SearchInstance> createSearchInstance(
           Solver *solver, HistorySequence *sequence, long maximumDepth) override;

    virtual StrategyInfo *sampleAStrategy(
            std::unordered_set<long> strategiesToExclude = {});
    virtual void updateStrategyWeights(long strategyNo, double timeUsed,
            double deltaRootQValue);
  private:
    std::vector<StrategyInfo> strategies_;
    Model *model_;
    double strategyExplorationCoefficient_;
};

class MultipleStrategiesExp3Instance: public SearchInstance {
  public:
    MultipleStrategiesExp3Instance(MultipleStrategiesExp3 *parent,
            Solver *solver, HistorySequence *sequence, long maximumDepth);
    virtual ~MultipleStrategiesExp3Instance();
    _NO_COPY_OR_MOVE(MultipleStrategiesExp3Instance);

    virtual std::pair<SearchStatus, std::unique_ptr<Action>>
    getStatusAndNextAction() override;
    virtual SearchStatus initialize();
    virtual SearchStatus finalize();
  private:
    MultipleStrategiesExp3 *parent_;
    std::unordered_set<long> failedStrategies_;

    long currentStrategyNo_;
    std::unique_ptr<SearchInstance> currentInstance_;
    std::clock_t currentInstanceStartTime_;
    double initialRootQValue_;
};

} /* namespace solver */

#endif /* SOLVER_MULTIPLESTRATEGIESEXP3_HPP_ */
