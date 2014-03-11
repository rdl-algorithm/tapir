#ifndef SOLVER_MULTIPLESTRATEGIESEXP3_HPP_
#define SOLVER_MULTIPLESTRATEGIESEXP3_HPP_

#include "SearchStatus.hpp"
#include "SearchStrategy.hpp"

namespace solver {
class MultipleStrategiesExp3: public SearchStrategy {
    friend class MultipleStrategiesExp3Instance;
  public:
    MultipleStrategiesExp3(std::vector<std::unique_ptr<SearchStrategy>> strategies,
            double strategyExplorationCoefficient);
    virtual ~MultipleStrategiesExp3() = default;

    virtual std::unique_ptr<SearchInstance> createSearchInstance(
           Solver *solver, HistorySequence *sequence);
  private:
    std::vector<StrategyInfo> strategies_;
    double strategyExplorationCoefficient_;
};

struct StrategyInfo {
    std::unique_ptr<SearchStrategy> strategy;
    double timeSpent;
    double weight;
    double probability;
    long numberOfTimesUsed;
};

class MultipleStrategiesExp3Instance: public SearchInstance {
  public:
    MultipleStrategiesExp3Instance(MultipleStrategiesExp3 *parent,
            long strategyNo, std::unique_ptr<SearchInstance> searchInstance);

    virtual std::pair<SearchStatus, std::unique_ptr<Action>> getStatusAndNextAction() = 0;
  private:
    MultipleStrategiesExp3 *parent_;
    long strategyNo_;
    std::unique_ptr<SearchInstance> searchInstance_;
}; /* namespace solver */

#endif /* SOLVER_MULTIPLESTRATEGIESEXP3_HPP_ */
