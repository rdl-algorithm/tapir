#ifndef SOLVER_RANDOMROLLOUTSTRATEGY_HPP_
#define SOLVER_RANDOMROLLOUTSTRATEGY_HPP_

#include "SearchStatus.hpp"
#include "SearchStrategy.hpp"

namespace solver {
class RandomRolloutStrategy: public SearchStrategy {
  public:
    RandomRolloutStrategy(long maxNSteps);
    virtual ~RandomRolloutStrategy() = default;

    virtual std::unique_ptr<SearchInstance> createSearchInstance(
           Solver *solver, HistorySequence *sequence, long maximumDepth) override;
  private:
    long maxNSteps_;
};

class RandomRolloutInstance: public SearchInstance {
  public:
    RandomRolloutInstance(long maxNSteps,
            Solver *solver, HistorySequence *sequence, long maximumDepth);

    virtual std::pair<SearchStatus, std::unique_ptr<Action>>
    getStatusAndNextAction() override;
  private:
    long maxNSteps_;
    long currentNSteps_;
};

} /* namespace solver */

#endif /* SOLVER_RANDOMROLLOUTSTRATEGY_HPP_ */
