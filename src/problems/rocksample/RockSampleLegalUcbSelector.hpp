#ifndef SOLVER_ROCKSAMPLE_LEGALUCBSELECTOR_HPP_
#define SOLVER_ROCKSAMPLE_LEGALUCBSELECTOR_HPP_

#include "solver/search/SearchStatus.hpp"
#include "solver/search/SearchStrategy.hpp"

#include "problems/shared/parsers.hpp"

namespace rocksample {
class RockSampleLegalUcbSelector: public solver::SearchStrategy {
  public:
    RockSampleLegalUcbSelector(solver::Solver *solver, double explorationCoefficient);
    virtual ~RockSampleLegalUcbSelector() = default;

    virtual std::unique_ptr<solver::SearchInstance> createSearchInstance(
            solver::HistorySequence *sequence, long maximumDepth) override;
  private:
    double explorationCoefficient_;
};

class RockSampleLegalUcbSelectorInstance: public solver::AbstractSearchInstance {
  public:
    RockSampleLegalUcbSelectorInstance(double explorationCoefficient,
            solver::Solver *solver, solver::HistorySequence *sequence,
            long maximumDepth);

    virtual std::pair<solver::SearchStatus, std::unique_ptr<solver::Action>>
    getStatusAndNextAction() override;
  private:
    double explorationCoefficient_;
};

class RockSampleLegalUcbSelectorParser: public Parser<solver::SearchStrategy> {
  public:
    RockSampleLegalUcbSelectorParser() = default;
    virtual ~RockSampleLegalUcbSelectorParser() = default;
    virtual std::unique_ptr<solver::SearchStrategy> parse(
            solver::Solver *solver,
            ParserSet<solver::SearchStrategy> *allParser,
            std::vector<std::string> args) override;
};

} /* namespace solver */

#endif /* SOLVER_ROCKSAMPLE_LEGALUCBSELECTOR_HPP_ */
