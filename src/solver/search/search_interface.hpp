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
            HistorySequence *sequence, SearchStatus &status) = 0;

    virtual Solver *getSolver() const;
  private:
    Solver *solver_;
};

class SearchInstance {
public:
    SearchInstance(SearchStatus &status) = default;
    virtual ~SearchInstance() = default;

    /**
     * Returns the result of an additional step in the simulation.
     */
    virtual Model::StepResult getStep() = 0;

    /** Returns the heuristic value for the current state. */
    virtual double getHeuristicValue() = 0;
protected:
    SearchStatus &status_;
};
} /* namespace solver */

#endif /* SOLVER_SEARCHSTRATEGY_HPP_ */
