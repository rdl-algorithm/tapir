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
    StepGenerator(SearchStatus &status);
    virtual ~StepGenerator() = default;
    _NO_COPY_OR_MOVE(StepGenerator);

    /** Returns the result of an additional step in the simulation;
     * a null action ends the simulation. */
    virtual Model::StepResult getStep() = 0;
protected:
    SearchStatus &status_;
};

class StepGeneratorFactory {
public:
    StepGeneratorFactory() = default;
    virtual ~StepGeneratorFactory() = default;

    /** Makes a new step generator for the given sequence. */
    virtual std::unique_ptr<StepGenerator> createGenerator(SearchStatus &status,
            HistorySequence *sequence) = 0;
};

class StagedStepGenerator: public StepGenerator {
public:
    StagedStepGenerator(SearchStatus &status, HistorySequence *sequence,
            std::vector<std::unique_ptr<StepGeneratorFactory>> const &generatorSequence);
    virtual ~StagedStepGenerator() = default;
    _NO_COPY_OR_MOVE(StagedStepGenerator);

    virtual Model::StepResult getStep() override;
private:
    HistorySequence *sequence_;
    std::vector<std::unique_ptr<StepGeneratorFactory>> const &generatorSequence_;
    std::vector<std::unique_ptr<StepGeneratorFactory>>::const_iterator iterator_;
    std::unique_ptr<StepGenerator> generator_;
};

class StagedStepGeneratorFactory : public StepGeneratorFactory {
public:
    StagedStepGeneratorFactory(
            std::vector<std::unique_ptr<StepGeneratorFactory>> generatorSequence);
    virtual ~StagedStepGeneratorFactory() = default;

    virtual std::unique_ptr<StepGenerator> createGenerator(SearchStatus &status,
                HistorySequence *sequence) override;
private:
    std::vector<std::unique_ptr<StepGeneratorFactory>> generatorSequence_;
};

class BasicSearchStrategy: public SearchStrategy {
public:
    BasicSearchStrategy(Solver *solver,
            std::unique_ptr<StepGeneratorFactory> generatorFactory,
            std::unique_ptr<Heuristic> heuristic);
    virtual ~BasicSearchStrategy() = default;
    _NO_COPY_OR_MOVE(BasicSearchStrategy);

    virtual std::unique_ptr<SearchInstance> createSearchInstance(SearchStatus &status,
            HistorySequence *sequence, long maximumDepth) override;
private:
    Solver *solver_;
    std::unique_ptr<StepGeneratorFactory> generatorFactory_;
    std::unique_ptr<Heuristic> heuristic_;
};

class BasicSearchInstance: public SearchInstance {
public:
    BasicSearchInstance(SearchStatus &status, HistorySequence *sequence, long maximumDepth,
            Solver *solver,
            std::unique_ptr<StepGenerator> generator,
            Heuristic *heuristic);
    virtual ~BasicSearchInstance() = default;
    _NO_COPY_OR_MOVE(BasicSearchInstance);

    virtual void extendSequence() override;
private:
    HistorySequence *sequence_;
    long maximumDepth_;
    Solver *solver_;
    Model *model_;
    std::unique_ptr<StepGenerator> generator_;
    Heuristic *heuristic_;
};
} /* namespace solver */

#endif /* SOLVER_SEARCHSTRATEGY_HPP_ */
