#ifndef ROCKSAMPLE_LEGALROLLOUTSTRATEGY_HPP_
#define ROCKSAMPLE_LEGALROLLOUTSTRATEGY_HPP_

#include "solver/search/SearchStrategy.hpp"
#include "problems/shared/strategy_parsers.hpp"

namespace solver {
    class HistorySequence;
    class Solver;
}

namespace rocksample {

class RockSampleLegalRolloutStrategy: public solver::SearchStrategy {
public:
    RockSampleLegalRolloutStrategy(solver::Solver *solver, long maxNSteps);
    virtual ~RockSampleLegalRolloutStrategy() = default;

    virtual std::unique_ptr<solver::SearchInstance> createSearchInstance(
                solver::HistorySequence *sequence,
                long maximumDepth) override;
private:
  long maxNSteps_;
};

class RockSampleLegalRolloutInstance: public solver::AbstractSearchInstance {
  public:
    RockSampleLegalRolloutInstance(long maxNSteps, solver::Solver *solver,
            solver::HistorySequence *sequence, long maximumDepth);

    virtual std::pair<solver::SearchStatus, std::unique_ptr<solver::Action>>
    getStatusAndNextAction() override;
  private:
    long maxNSteps_;
    long currentNSteps_;
};

class RockSampleLegalParser: public StrategyParser {
  public:
    RockSampleLegalParser() = default;
    virtual ~RockSampleLegalParser() = default;
    virtual std::unique_ptr<solver::SearchStrategy> parseStrategy(
            solver::Solver *solver, AllStrategiesParser *allParser,
            std::vector<std::string> args) override;
};

} /* namespace rocksample */

#endif /* ROCKSAMPLE_LEGALROLLOUTSTRATEGY_HPP_ */
