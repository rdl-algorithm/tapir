#ifndef SOLVER_SEARCHSTRATEGY_HPP_
#define SOLVER_SEARCHSTRATEGY_HPP_

#include <memory>

#include "solver/abstract-problem/Action.hpp"

#include "SearchStatus.hpp"

namespace solver {
class BeliefNode;
class HistoricalData;
class HistorySequence;
class Model;
class SearchInstance;
class Solver;

class SearchStrategy {
  public:
    SearchStrategy(Solver *solver);
    virtual ~SearchStrategy() = default;
    _NO_COPY_OR_MOVE(SearchStrategy);

    virtual std::unique_ptr<SearchInstance> createSearchInstance(
            HistorySequence *sequence, long maximumDepth) = 0;
  protected:
    Solver *solver_;
};

class SearchInstance {
  public:
    SearchInstance() = default;
    virtual ~SearchInstance() = default;
    /** Initializes the search. */
    virtual SearchStatus initialize() = 0;
    /** Extends the sequence. */
    virtual SearchStatus extendSequence() = 0;
    /** Finalizes the search. */
    virtual SearchStatus finalize() = 0;
};

struct SearchStep {
    SearchStatus status;
    std::unique_ptr<Action> action;
    bool createNode;
};

class AbstractSearchInstance : public SearchInstance {
  friend class AbstractSelectionInstance;
  friend class AbstractRolloutInstance;
  public:
    AbstractSearchInstance(Solver *solver, HistorySequence *sequence,
            long maximumDepth);
    virtual ~AbstractSearchInstance() = default;
    _NO_COPY_OR_MOVE(AbstractSearchInstance);

    /** The key method that defines how the search will proceed.
     * This method returns the current search status, and the next action
     * selected. If the selected action is null, the search will terminate.
     */
    virtual SearchStep getSearchStep() = 0;

    /** The default initialization method - cannot be overridden. */
    virtual SearchStatus initialize() final override;
    /** Initialization method, for overriding by subclasses. */
    virtual SearchStatus initializeCustom(BeliefNode *currentNode);
    /** A default implementation to handle extending sequences; this probably
     * shouldn't need to be changed.
     */
    virtual SearchStatus extendSequence();
    /** The default finalization method - cannot be overridden. */
    virtual SearchStatus finalize() final override;
    /** Finalization method, for overriding by subclasses. */
    virtual SearchStatus finalizeCustom();

  private:
    Solver *solver_;
    Model *model_;
    HistorySequence *sequence_;
    BeliefNode *currentNode_;
    std::unique_ptr<HistoricalData> currentHistoricalData_;
    double discountFactor_;
    long maximumDepth_;
    SearchStatus status_;
};

class AbstractSelectionInstance : public AbstractSearchInstance {
public:
    AbstractSelectionInstance(Solver *solver, HistorySequence *sequence,
            long maximumDepth);
    virtual ~AbstractSelectionInstance() = default;
    _NO_COPY_OR_MOVE(AbstractSelectionInstance);

    virtual SearchStep getSearchStep() override;
    virtual SearchStep getSearchStep(HistorySequence *sequence, BeliefNode *currentNode) = 0;
};

class AbstractRolloutInstance : public AbstractSearchInstance {
public:
    AbstractRolloutInstance(Solver *solver, HistorySequence *sequence,
            long maximumDepth);
    virtual ~AbstractRolloutInstance() = default;
    _NO_COPY_OR_MOVE(AbstractRolloutInstance);

    virtual SearchStep getSearchStep() override;
    virtual SearchStep getSearchStep(Solver *solver, HistorySequence *sequence, HistoricalData *data) = 0;
};

} /* namespace solver */

#endif /* SOLVER_SEARCHSTRATEGY_HPP_ */
