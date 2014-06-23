#ifndef SOLVER_SEARCHSTRATEGY_HPP_
#define SOLVER_SEARCHSTRATEGY_HPP_

#include <memory>

#include "global.hpp"

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/Model.hpp"

#include "solver/abstract-problem/heuristics/Heuristic.hpp"

#include "solver/search/SearchStatus.hpp"

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

    virtual SearchStatus extendSequence(HistorySequence *sequence, long maximumDepth,
            bool doBackup) = 0;
};

class StepGeneratorFactory {
public:
    StepGeneratorFactory() = default;
    virtual ~StepGeneratorFactory() = default;

    virtual std::unique_ptr<StepGenerator> createGenerator(SearchStatus &status,
            HistoryEntry const *entry, State const *state, HistoricalData const *data) = 0;
};

class StepGenerator {
public:
    StepGenerator(SearchStatus &status);
    virtual ~StepGenerator() = default;

    /** Returns the result of an additional step in the simulation;
     * a null action ends the simulation. */
    virtual Model::StepResult getStep(HistoryEntry const *entry, State const *state,
            HistoricalData const *data) = 0;
protected:
    SearchStatus &status_;
};

class StagedStepGeneratorFactory: public StepGeneratorFactory {
public:
    StagedStepGeneratorFactory(std::vector<std::unique_ptr<StepGeneratorFactory>> factories);
    virtual ~StagedStepGeneratorFactory() = default;
    _NO_COPY_OR_MOVE(StagedStepGeneratorFactory);

    virtual std::unique_ptr<StepGenerator> createGenerator(SearchStatus &status,
            HistoryEntry const *entry, State const *state, HistoricalData const *data);

private:
    std::vector<std::unique_ptr<StepGeneratorFactory>> factories_;
};

class StagedStepGenerator: public StepGenerator {
public:
    StagedStepGenerator(SearchStatus &status,
            std::vector<std::unique_ptr<StepGeneratorFactory>> const &factories,
            HistoryEntry const *entry, State const *state, HistoricalData const *data);
    virtual ~StagedStepGenerator() = default;
    _NO_COPY_OR_MOVE(StagedStepGenerator);

    virtual Model::StepResult getStep(HistoryEntry const *entry, State const *state,
            HistoricalData const *data) override;

private:
    std::vector<std::unique_ptr<StepGeneratorFactory>> const &factories_;
    std::vector<std::unique_ptr<StepGeneratorFactory>>::const_iterator iterator_;
    std::unique_ptr<StepGenerator> generator_;
};

class BasicSearchStrategy: public SearchStrategy {
public:
    BasicSearchStrategy(Solver *solver, std::unique_ptr<StepGeneratorFactory> factory,
            Heuristic heuristic);
    virtual ~BasicSearchStrategy() = default;
    _NO_COPY_OR_MOVE(BasicSearchStrategy);

    virtual SearchStatus extendSequence(HistorySequence *sequence, long maximumDepth,
            bool doBackup) override;
private:
    Solver *solver_;
    std::unique_ptr<StepGeneratorFactory> factory_;
    Heuristic heuristic_;
};

} /* namespace solver */

#endif /* SOLVER_SEARCHSTRATEGY_HPP_ */
