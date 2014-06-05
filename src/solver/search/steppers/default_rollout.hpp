#ifndef SOLVER_DEFAULTROLLOUTSTRATEGY_HPP_
#define SOLVER_DEFAULTROLLOUTSTRATEGY_HPP_

#include "solver/search/SearchStatus.hpp"
#include "solver/search/search_interface.hpp"

namespace solver {
class HistoricalData;
class HistorySequence;
class Solver;

class DefaultRolloutFactory: public StepGeneratorFactory {
public:
    DefaultRolloutFactory(Solver *solver, long maxNSteps);
    virtual ~DefaultRolloutFactory() = default;
    _NO_COPY_OR_MOVE(DefaultRolloutFactory);

    virtual std::unique_ptr<StepGenerator> createGenerator(SearchStatus &status,
            HistorySequence *sequence) override;
private:
    Solver *solver_;
    long maxNSteps_;
};

class DefaultRolloutGenerator: public StepGenerator {
public:
    DefaultRolloutGenerator(SearchStatus &status, HistorySequence *sequence,
            Solver *solver, long maxNSteps);
    virtual ~DefaultRolloutGenerator() = default;
    _NO_COPY_OR_MOVE(DefaultRolloutGenerator);

    virtual Model::StepResult getStep(HistoryEntry const *entry,
            State const *state, HistoricalData const *data) override;

private:
    HistorySequence *sequence_;
    Solver *solver_;
    Model *model_;
    long maxNSteps_;
    long currentNSteps_;
};

} /* namespace solver */

#endif /* SOLVER_DEFAULTROLLOUTSTRATEGY_HPP_ */
