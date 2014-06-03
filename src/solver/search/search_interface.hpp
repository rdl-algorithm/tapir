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
class StepGenerator;

class SearchStrategy {
public:
    SearchStrategy() = default;
    virtual ~SearchStrategy() = default;
    _NO_COPY_OR_MOVE(SearchStrategy);

    virtual std::unique_ptr<SearchInstance> createSearchInstance(SearchStatus &status,
            HistorySequence *sequence,
            long maximumDepth) = 0;
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


class StepGeneratorFactory {
public:
    StepGeneratorFactory() = default;
    virtual ~StepGeneratorFactory() = default;

    /** Makes a new step generator for the given sequence. */
    virtual std::unique_ptr<StepGenerator> createGenerator(SearchStatus &status,
            HistorySequence *sequence);
};

class StepGenerator {
public:
    StepGenerator(SearchStatus &status);
    virtual ~StepGenerator() = default;
    _NO_COPY_OR_MOVE(StepGenerator);

    /** Returns the result of an additional step in the simulation;
     * a null action ends the simulation. */
    virtual Model::StepResult getStep() = 0;
protected:
    SearchStatus &status_;
};

class StagedSearchStrategy : public SearchStrategy {
public:
    StagedSearchStrategy(Solver *solver,
            std::vector<std::unique_ptr<StepGeneratorFactory>> generatorFactories,
            std::function<double(HistoryEntry const *)> heuristic);
    virtual ~StagedSearchStrategy() = default;
    _NO_COPY_OR_MOVE(StagedSearchStrategy);

    virtual std::unique_ptr<SearchInstance> createSearchInstance(SearchStatus &status,
            HistorySequence *sequence, long maximumDepth) override;
private:
    Solver *solver_;
    std::vector<std::unique_ptr<StepGeneratorFactory>> generatorFactories_;
    std::function<double(HistoryEntry const *)> heuristic_;
};

class StagedSearchInstance : public SearchInstance {
    StagedSearchInstance(SearchStatus &status,
            HistorySequence *sequence, long maximumDepth,
            Solver *solver,
            std::vector<std::unique_ptr<StepGeneratorFactory>> const &generators,
            std::function<double(HistoryEntry const *)> heuristic);
    virtual ~StagedSearchInstance() = default;
    _NO_COPY_OR_MOVE(StagedSearchInstance);

    virtual void extendSequence() override;
private:
    HistorySequence *sequence_;
    long maximumDepth_;
    Solver *solver_;
    Model *model_;
    std::vector<std::unique_ptr<StepGeneratorFactory>> const &generators_;
    std::vector<std::unique_ptr<StepGeneratorFactory>>::const_iterator iterator_;
    std::unique_ptr<StepGenerator> generator_;
    std::function<double(HistoryEntry const *)> heuristic_;
};
} /* namespace solver */

#endif /* SOLVER_SEARCHSTRATEGY_HPP_ */
