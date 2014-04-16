#ifndef SOLVER_NNROLLOUTSTRATEGY_HPP_
#define SOLVER_NNROLLOUTSTRATEGY_HPP_

#include <unordered_map>

#include "SearchStatus.hpp"
#include "SearchStrategy.hpp"

namespace solver {
struct NnData {
    double tNnComp = -1;
    BeliefNode *neighbor = nullptr;
};

class NnRolloutStrategy: public SearchStrategy {
  public:
    NnRolloutStrategy(Solver *solver,
            long maxNnComparisons, double maxNnDistance);
    virtual ~NnRolloutStrategy() = default;

    virtual BeliefNode* findNeighbor(BeliefNode *beliefNode);

    virtual std::unique_ptr<SearchInstance> createSearchInstance(
            HistorySequence *sequence, long maximumDepth) override;
  private:
    long maxNnComparisons_;
    double maxNnDistance_;
    std::unordered_map<BeliefNode *, NnData> nnMap_;
};

class NnRolloutInstance: public AbstractRolloutInstance {
  public:
    NnRolloutInstance(NnRolloutStrategy *strategy,
            Solver *solver, HistorySequence *sequence, long maximumDepth);
    virtual ~NnRolloutInstance() = default;
    _NO_COPY_OR_MOVE(NnRolloutInstance);

    virtual SearchStatus initializeCustom(BeliefNode *currentNode) override;
    virtual SearchStep getSearchStep(HistoricalData *historicalData) override;
  private:
    NnRolloutStrategy *strategy_;
    BeliefNode *rootNeighborNode_;
    BeliefNode *currentNeighborNode_;
    std::unique_ptr<Action> previousAction_;
};

} /* namespace solver */

#endif /* SOLVER_NNROLLOUTSTRATEGY_HPP_ */
