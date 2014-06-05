#ifndef SOLVER_SEARCHSTRATEGY_HPP_
#define SOLVER_SEARCHSTRATEGY_HPP_

#include <memory>

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/Model.hpp"

#include "SearchStatus.hpp"

namespace solver {
class BeliefNode;
class Heuristic;
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
            HistorySequence *sequence, long maximumDepth) = 0;
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
    StepGenerator();
    virtual ~StepGenerator() = default;
    _NO_COPY_OR_MOVE(StepGenerator);

    virtual void initialise(SearchStatus &status,
            HistoryEntry const *entry,
            State const *state,
            HistoricalData const *data);

    /** Returns the result of an additional step in the simulation;
     * a null action ends the simulation. */
    virtual Model::StepResult getStep(HistoryEntry const *entry,
            State const *state, HistoricalData const *data) = 0;
protected:
    SearchStatus *status_;
};

class StagedStepGenerator: public StepGenerator {
public:
    StagedStepGenerator(std::vector<std::unique_ptr<StepGenerator>> const &generators);
    virtual ~StagedStepGenerator() = default;
    _NO_COPY_OR_MOVE(StagedStepGenerator);

    virtual void initialise(SearchStatus &status, HistoryEntry const *entry,
            State const *state,
            HistoricalData const *data);

    virtual Model::StepResult getStep(HistoryEntry const *entry,
            State const *state, HistoricalData const *data) override;
private:
    std::vector<std::unique_ptr<StepGenerator>> const &generators;
    std::vector<std::unique_ptr<StepGenerator>>::const_iterator iterator_;
    StepGenerator *generator_;
};

class BasicSearchStrategy: public SearchStrategy {
public:
    BasicSearchStrategy(Solver *solver,
            std::unique_ptr<StepGenerator> generators,
            std::unique_ptr<Heuristic> heuristic);
    virtual ~BasicSearchStrategy() = default;
    _NO_COPY_OR_MOVE(BasicSearchStrategy);

    virtual std::unique_ptr<SearchInstance> createSearchInstance(SearchStatus &status,
            HistorySequence *sequence, long maximumDepth) override;
private:
    Solver *solver_;
    std::unique_ptr<StepGenerator> generators_;
    std::unique_ptr<Heuristic> heuristic_;
};

class BasicSearchInstance: public SearchInstance {
public:
    BasicSearchInstance(SearchStatus &status, HistorySequence *sequence, long maximumDepth,
            Solver *solver,
            std::unique_ptr<StepGenerator> const &generators,
            Heuristic *heuristic);
    virtual ~BasicSearchInstance() = default;
    _NO_COPY_OR_MOVE(BasicSearchInstance);

    virtual void extendSequence() override;
private:
    HistorySequence *sequence_;
    long maximumDepth_;
    Solver *solver_;
    Model *model_;
    StepGenerator *generator_;
    Heuristic *heuristic_;
};
} /* namespace solver */

#endif /* SOLVER_SEARCHSTRATEGY_HPP_ */
