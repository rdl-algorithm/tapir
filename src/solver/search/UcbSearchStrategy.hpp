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
           Solver *solver,
           BeliefNode *currentNode, HistorySequence *sequence,
           double discountFactor, long maximumDepth);
  private:
    double explorationCoefficient_;
};

class UcbSearchInstance: public SearchInstance {
  public:
    UcbSearchInstance(double explorationCoefficient,
            Solver *solver,
            BeliefNode *currentNode, HistorySequence *sequence,
            double discountFactor, long maximumDepth);

    virtual std::unique_ptr<Action> getNextAction();
    virtual SearchStatus getStatus();
  private:
    double explorationCoefficient_;
}; /* namespace solver */

#endif /* SOLVER_UCBSEARCHSTRATEGY_HPP_ */
