#ifndef SOLVER_SEARCHSTRATEGY_HPP_
#define SOLVER_SEARCHSTRATEGY_HPP_

#include <memory>

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/Model.hpp"

#include "SearchStatus.hpp"

namespace solver {
class BeliefNode;
class HistoricalData;
class HistoryEntry;
class HistorySequence;
class SearchInstance;
class Solver;

class SearchStrategy {
public:
    SearchStrategy(Solver *solver);
    virtual ~SearchStrategy() = default;
    _NO_COPY_OR_MOVE(SearchStrategy);

    virtual std::unique_ptr<SearchInstance> createSearchInstance(SearchStatus &status,
            HistorySequence *sequence, long maximumDepth) = 0;

    virtual Solver *getSolver() const;
private:
    Solver *solver_;
};

class SearchInstance {
public:
    SearchInstance(SearchStatus &status);
    virtual ~SearchInstance() = default;
    _NO_COPY_OR_MOVE(SearchInstance);

    virtual void extendSequence() = 0;
protected:
    SearchStatus &status_;
};

class StepGenerator {
public:
    StepGenerator() = default;
    virtual ~StepGenerator() = default;

    /** Returns the result of an additional step in the simulation. */
    virtual Model::StepResult getStep() = 0;
};

class BasicSearchInstance: public SearchInstance {
public:
    BasicSearchInstance(SearchStatus &status, HistorySequence *sequence, long maximumDepth,
            Solver *solver, std::unique_ptr<StepGenerator> stepGenerator,
            std::function<double(HistoryEntry const *)> heuristic);
    virtual ~BasicSearchInstance() = default;
    _NO_COPY_OR_MOVE(BasicSearchInstance);

    virtual void extendSequence() override;
private:
    HistorySequence *sequence_;
    long maximumDepth_;
    Solver *solver_;
    Model *model_;
    std::unique_ptr<StepGenerator> stepGenerator_;
    std::function<double(HistoryEntry const *)> heuristic_;
};
} /* namespace solver */

#endif /* SOLVER_SEARCHSTRATEGY_HPP_ */
