#ifndef SOLVER_DEFAULTROLLOUTSTRATEGY_HPP_
#define SOLVER_DEFAULTROLLOUTSTRATEGY_HPP_

#include "SearchStatus.hpp"
#include "search_interface.hpp"

namespace solver {
class HistoricalData;
class HistorySequence;
class Solver;

class DefaultRolloutGenerator : public StepGenerator {
public:
    DefaultRolloutGenerator(HistorySequence *sequence, Solver *solver, long maxNSteps);
    virtual ~DefaultRolloutGenerator() = default;

    virtual Model::StepResult getStep() override;

    private:
    HistorySequence *sequence_;
    Solver *solver_;
    long maxNSteps_;
    long currentNSteps_;

};

class DefaultRolloutStrategy: public SearchStrategy {
  public:
    DefaultRolloutStrategy(Solver *solver, long maxNSteps);
    virtual ~DefaultRolloutStrategy() = default;

    virtual std::unique_ptr<SearchInstance> createSearchInstance(SearchStatus &status,
            HistorySequence *sequence, long maximumDepth) override;
  private:
    long maxNSteps_;
};

} /* namespace solver */

#endif /* SOLVER_DEFAULTROLLOUTSTRATEGY_HPP_ */
