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
           Solver *solver, HistorySequence *sequence, long maximumDepth) override;
  private:
    long maxNnComparisons_;
    double maxNnDistance_;
};

class NnRolloutInstance: public SearchInstance {
  public:
    NnRolloutInstance(long maxNnComparisons, double maxNnDistance,
            Solver *solver, HistorySequence *sequence, long maximumDepth);
    virtual ~NnRolloutInstance() = default;
    _NO_COPY_OR_MOVE(NnRolloutInstance);

    virtual SearchStatus initialize() override;
    virtual std::pair<SearchStatus, std::unique_ptr<Action>>
    getStatusAndNextAction() override;
  private:
    long maxNnComparisons_;
    double maxNnDistance_;
    BeliefNode *rootNeighborNode_;
    BeliefNode *currentNeighborNode_;
    std::unique_ptr<Action> previousAction_;
};

} /* namespace solver */

#endif /* SOLVER_NNROLLOUTSTRATEGY_HPP_ */
