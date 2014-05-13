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

    virtual Solver *getSolver() const;
  private:
    Solver *solver_;
};

class SearchInstance {
  public:
    SearchInstance() = default;
    virtual ~SearchInstance() = default;

    /** Returns the current search status. */
    virtual SearchStatus getStatus() const = 0;
    /** Extends the sequence. */
    virtual void extendSequence() = 0;
};

/*
class StagedSearchStrategy : SearchStrategy {
    StagedSearchStrategy(Solver *solver, std::unique_ptr<SearchStrategy> searchStrategy,
            std::unique_ptr<SearchStrategy> rolloutStrategy,
            std::unique_ptr<ValueEstimator> estimator);
    virtual ~StagedSearchStrategy() = default;

    virtual std::unique_ptr<SearchInstance> createSearchInstance(
               HistorySequence *sequence, long maximumDepth);
};
*/


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

    virtual SearchStatus initialize(BeliefNode *currentNode);
    virtual SearchStatus finalize();

    virtual SearchStatus getStatus() const override;
    virtual void extendSequence() override;

    /** Returns the solver associated with this search instance. */
    virtual Solver *getSolver() const;
    /** Returns the history sequence associated with this search instance. */
    virtual HistorySequence *getSequence() const;
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
    virtual SearchStep getSearchStep(BeliefNode *currentNode) = 0;
};

class AbstractRolloutInstance : public AbstractSearchInstance {
public:
    AbstractRolloutInstance(Solver *solver, HistorySequence *sequence,
            long maximumDepth);
    virtual ~AbstractRolloutInstance() = default;
    _NO_COPY_OR_MOVE(AbstractRolloutInstance);

    virtual SearchStep getSearchStep() override;
    virtual SearchStep getSearchStep(HistoricalData *data) = 0;
};

} /* namespace solver */

#endif /* SOLVER_SEARCHSTRATEGY_HPP_ */
