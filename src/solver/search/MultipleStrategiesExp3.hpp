#ifndef SOLVER_MULTIPLESTRATEGIESEXP3_HPP_
#define SOLVER_MULTIPLESTRATEGIESEXP3_HPP_

#include <ctime>

#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

#include "SearchStatus.hpp"
#include "search_interface.hpp"

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
    MultipleStrategiesExp3(Solver *solver,
            double strategyExplorationCoefficient,
            std::vector<std::unique_ptr<SearchStrategy>> strategies);
    virtual ~MultipleStrategiesExp3() = default;
    _NO_COPY_OR_MOVE(MultipleStrategiesExp3);

    virtual std::unique_ptr<SearchInstance> createSearchInstance(
           HistorySequence *sequence, long maximumDepth) override;

    virtual StrategyInfo *sampleAStrategy(
            std::unordered_set<long> strategiesToExclude = std::unordered_set<long> {});
    virtual void updateStrategyWeights(long strategyNo, double timeUsed,
            double deltaRootQValue);
  private:
    double strategyExplorationCoefficient_;
    std::vector<StrategyInfo> strategies_;
    Model *model_;
};

class MultipleStrategiesExp3Instance: public SearchInstance {
  public:
    MultipleStrategiesExp3Instance(MultipleStrategiesExp3 *parent,
            Solver *solver, HistorySequence *sequence, long maximumDepth);
    virtual ~MultipleStrategiesExp3Instance() = default;
    _NO_COPY_OR_MOVE(MultipleStrategiesExp3Instance);

    virtual SearchStatus getStatus() const override;
    virtual void extendSequence() override;
  private:
    MultipleStrategiesExp3 *parent_;
    Solver *solver_;
    HistorySequence *sequence_;
    long maximumDepth_;

    SearchStatus status_;
};

} /* namespace solver */

#endif /* SOLVER_MULTIPLESTRATEGIESEXP3_HPP_ */
