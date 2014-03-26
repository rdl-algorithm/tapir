#ifndef SOLVER_UCBSEARCHSTRATEGY_HPP_
#define SOLVER_UCBSEARCHSTRATEGY_HPP_

#include "SearchStatus.hpp"
#include "SearchStrategy.hpp"

namespace solver {
class UcbSelectionStrategy: public SearchStrategy {
  public:
    UcbSelectionStrategy(Solver *solver, double explorationCoefficient);
    virtual ~UcbSelectionStrategy() = default;

    virtual std::unique_ptr<SearchInstance> createSearchInstance(
            HistorySequence *sequence, long maximumDepth) override;
  private:
    double explorationCoefficient_;
};

class UcbSelectionInstance: public AbstractSearchInstance {
  public:
    UcbSelectionInstance(double explorationCoefficient,
            Solver *solver, HistorySequence *sequence,
            long maximumDepth);

    virtual std::pair<SearchStatus, std::unique_ptr<Action>>
    getStatusAndNextAction() override;
  private:
    double explorationCoefficient_;
};

} /* namespace solver */

#endif /* SOLVER_UCBSEARCHSTRATEGY_HPP_ */
