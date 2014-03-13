#ifndef SOLVER_SEARCHSTRATEGY_HPP_
#define SOLVER_SEARCHSTRATEGY_HPP_

#include <memory>

#include "solver/abstract-problem/Action.hpp"

#include "SearchStatus.hpp"

namespace solver {
class BeliefNode;
class HistorySequence;
class Model;
class SearchInstance;
class Solver;

class SearchStrategy {
  public:
    SearchStrategy() = default;
    virtual ~SearchStrategy() = default;

    virtual std::unique_ptr<SearchInstance> createSearchInstance(
           Solver *solver, HistorySequence *sequence, long maximumDepth) = 0;
};

class SearchInstance {
  public:
    /** Initializes the search. */
    virtual SearchStatus initialize() = 0;
    /** Extends the sequence. */
    virtual SearchStatus extendSequence() = 0;
    /** Finalizes the search. */
    virtual SearchStatus finalize() = 0;
};

class AbstractSearchInstance : public SearchInstance {
  public:
    AbstractSearchInstance(Solver *solver, HistorySequence *sequence,
            long maximumDepth);
    virtual ~AbstractSearchInstance() = default;
    _NO_COPY_OR_MOVE(AbstractSearchInstance);

    /** The key method that defines how the search will proceed.
     * This method returns the current search status, and the next action
     * selected. If the selected action is null, the search will terminate.
     */
    virtual std::pair<SearchStatus, std::unique_ptr<Action>> getStatusAndNextAction() = 0;
    virtual SearchStatus initialize();
    /** A default implementation to handle extending sequences; this probably
     * shouldn't need to be changed.
     */
    virtual SearchStatus extendSequence();
    virtual SearchStatus finalize();
  protected:
    Solver *solver_;
    Model *model_;
    HistorySequence *sequence_;
    BeliefNode *currentNode_;
    double discountFactor_;
    long maximumDepth_;
    SearchStatus status_;
};

} /* namespace solver */

#endif /* SOLVER_SEARCHSTRATEGY_HPP_ */
