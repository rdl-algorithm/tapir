#ifndef SOLVER_NNROLLOUTSTRATEGY_HPP_
#define SOLVER_NNROLLOUTSTRATEGY_HPP_

#include "SearchStatus.hpp"
#include "SearchStrategy.hpp"

namespace solver {
class NnRolloutStrategy: public SearchStrategy {
  public:
    NnRolloutStrategy(long maxNnComparisons, double maxNnDistance);
    virtual ~NnRolloutStrategy() = default;

    virtual SearchStatus extendSequence(BeliefNode *currentNode,
            HistorySequence *sequence,
            double discountFactor,
            long maximumDepth);
  private:
    long maxNnComparisons_;
    double maxNnDistance_;
};

class NnRolloutInstance: public SearchInstance {
  public:
    NnRolloutInstance(double explorationCoefficient,
            Solver *solver,
            BeliefNode *currentNode, HistorySequence *sequence,
            double discountFactor, long maximumDepth);

    virtual std::unique_ptr<Action> getNextAction();
    virtual SearchStatus getStatus();
  private:
    BeliefNode *neighborNode_;
}; /* namespace solver */

#endif /* SOLVER_NNROLLOUTSTRATEGY_HPP_ */
