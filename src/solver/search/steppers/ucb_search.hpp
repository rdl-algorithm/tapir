#ifndef SOLVER_UCB_SEARCH_HPP_
#define SOLVER_UCB_SEARCH_HPP_

#include "solver/search/SearchStatus.hpp"
#include "solver/search/search_interface.hpp"

namespace solver {
class UcbStepGeneratorFactory: public StepGeneratorFactory {
public:
    UcbStepGeneratorFactory(Solver *solver, double explorationCoefficient);
    virtual ~UcbStepGeneratorFactory() = default;
    _NO_COPY_OR_MOVE(UcbStepGeneratorFactory);

    virtual std::unique_ptr<StepGenerator> createGenerator(SearchStatus &status,
            HistoryEntry const *entry, State const *state, HistoricalData const *data) override;
private:
    Solver *solver_;
    double explorationCoefficient_;
};

class UcbStepGenerator : public StepGenerator {
public:
    UcbStepGenerator(SearchStatus &status, Solver *solver, double explorationCoefficient);
    ~UcbStepGenerator() = default;
    _NO_COPY_OR_MOVE(UcbStepGenerator);

    virtual Model::StepResult getStep(HistoryEntry const *entry,
            State const *state, HistoricalData const *data) override;
private:
    Model *model_;
    double explorationCoefficient_;

    bool choseUnvisitedAction_;
};
} /* namespace solver */

#endif /* SOLVER_UCB_SEARCH_HPP_ */
