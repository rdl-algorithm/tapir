#ifndef SOLVER_RANDOMROLLOUTSTRATEGY_HPP_
#define SOLVER_RANDOMROLLOUTSTRATEGY_HPP_

#include "SearchStatus.hpp"
#include "SearchStrategy.hpp"

namespace solver {
class HistoricalData;
class HistorySequence;
class Solver;

class RandomRolloutStrategy: public SearchStrategy {
  public:
    RandomRolloutStrategy(Solver *solver, long maxNSteps);
    virtual ~RandomRolloutStrategy() = default;

    virtual std::unique_ptr<SearchInstance> createSearchInstance(
            HistorySequence *sequence, long maximumDepth) override;
  private:
    long maxNSteps_;
};

class RandomRolloutInstance: public AbstractRolloutInstance {
  public:
    RandomRolloutInstance(long maxNSteps,
            Solver *solver, HistorySequence *sequence, long maximumDepth);

    virtual SearchStep getSearchStep(Solver *solver, HistorySequence *sequence,
            HistoricalData *currentData) override;
  private:
    long maxNSteps_;
    long currentNSteps_;
};

} /* namespace solver */

#endif /* SOLVER_RANDOMROLLOUTSTRATEGY_HPP_ */
