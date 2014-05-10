#ifndef SOLVER_DEFAULTROLLOUTSTRATEGY_HPP_
#define SOLVER_DEFAULTROLLOUTSTRATEGY_HPP_

#include "SearchStatus.hpp"
#include "SearchStrategy.hpp"

namespace solver {
class HistoricalData;
class HistorySequence;
class Solver;

class DefaultRolloutStrategy: public SearchStrategy {
  public:
    DefaultRolloutStrategy(Solver *solver, long maxNSteps);
    virtual ~DefaultRolloutStrategy() = default;

    virtual std::unique_ptr<SearchInstance> createSearchInstance(
            HistorySequence *sequence, long maximumDepth) override;
  private:
    long maxNSteps_;
};

class DefaultRolloutInstance: public AbstractRolloutInstance {
  public:
    DefaultRolloutInstance(long maxNSteps,
            Solver *solver, HistorySequence *sequence, long maximumDepth);

    virtual SearchStep getSearchStep(HistoricalData *currentData) override;
  private:
    long maxNSteps_;
    long currentNSteps_;
};

} /* namespace solver */

#endif /* SOLVER_DEFAULTROLLOUTSTRATEGY_HPP_ */
