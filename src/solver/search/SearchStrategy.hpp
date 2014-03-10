#ifndef SOLVER_SEARCHSTRATEGY_HPP_
#define SOLVER_SEARCHSTRATEGY_HPP_

#include <memory>

#include "solver/abstract-problem/Action.hpp"

#include "SearchStatus.hpp"

namespace solver {
class BeliefNode;
class HistorySequence;
class SearchInstance;

class SearchStrategy {
  public:
    SearchStrategy() = default;
    virtual ~SearchStrategy() = default;

    virtual std::unique_ptr<SearchInstance> createSearchInstance(
           Solver *solver,
           BeliefNode *currentNode, HistorySequence *sequence,
           double discountFactor, long maximumDepth) = 0;
};

class SearchInstance {
  public:
    SearchInstance(Solver *solver,
            BeliefNode *currentNode, HistorySequence *sequence,
            double discountFactor, long maximumDepth);
    virtual ~SearchInstance() = default;
    _NO_COPY_OR_MOVE(SearchInstance);

    /** The key method that defines how the search will proceed.
     * This method returns the current search status, and the next action
     * selected. If the selected action is null, the search will terminate.
     */
    virtual std::pair<SearchStatus, std::unique_ptr<Action>> getStatusAndNextAction() = 0;
    virtual SearchStatus extendSequence();
  protected:
    Solver *solver_;
    Model *model_;
    BeliefNode *currentNode_;
    HistorySequence *sequence_;
    double discountFactor_;
    long maximumDepth_;
};

} /* namespace solver */

#endif /* SOLVER_SearchStrategy_HPP_ */
