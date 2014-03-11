#ifndef SOLVER_UCBSEARCHSTRATEGY_HPP_
#define SOLVER_UCBSEARCHSTRATEGY_HPP_

#include "SearchStatus.hpp"
#include "SearchStrategy.hpp"

namespace solver {
class UcbSearchStrategy: public SearchStrategy {
  public:
    UcbSearchStrategy(double explorationCoefficient);
    virtual ~UcbSearchStrategy() = default;

    virtual std::unique_ptr<SearchInstance> createSearchInstance(
           Solver *solver, HistorySequence *sequence);
  private:
    double explorationCoefficient_;
};

class UcbSearchInstance: public SearchInstance {
  public:
    UcbSearchInstance(double explorationCoefficient,
            Solver *solver, HistorySequence *sequence);

    virtual std::pair<SearchStatus, std::unique_ptr<Action>> getStatusAndNextAction() = 0;
  private:
    double explorationCoefficient_;
}; /* namespace solver */

#endif /* SOLVER_UCBSEARCHSTRATEGY_HPP_ */
