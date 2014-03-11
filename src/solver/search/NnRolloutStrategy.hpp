#ifndef SOLVER_NNROLLOUTSTRATEGY_HPP_
#define SOLVER_NNROLLOUTSTRATEGY_HPP_

#include "SearchStatus.hpp"
#include "SearchStrategy.hpp"

namespace solver {
class NnRolloutStrategy: public SearchStrategy {
  public:
    NnRolloutStrategy(long maxNnComparisons, double maxNnDistance);
    virtual ~NnRolloutStrategy() = default;

    virtual std::unique_ptr<SearchInstance> createSearchInstance(
           Solver *solver, HistorySequence *sequence) = 0;
  private:
    long maxNnComparisons_;
    double maxNnDistance_;
};

class NnRolloutInstance: public SearchInstance {
  public:
    NnRolloutInstance(long maxNnComparisons, double maxNnDistance,
            Solver *solver, HistorySequence *sequence);

    virtual std::pair<SearchStatus, std::unique_ptr<Action>> getStatusAndNextAction() = 0;
  private:
    BeliefNode *rootNeighborNode_;
    BeliefNode *currentNeighborNode_;
    std::unique_ptr<Action> previousAction_;
}; /* namespace solver */

#endif /* SOLVER_NNROLLOUTSTRATEGY_HPP_ */
