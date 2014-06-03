#ifndef SOLVER_UCB_SEARCH_HPP_
#define SOLVER_UCB_SEARCH_HPP_

#include "SearchStatus.hpp"
#include "search_interface.hpp"

namespace solver {
class UcbStepGeneratorFactory: public StepGeneratorFactory {
public:
    UcbStepGeneratorFactory(Solver *solver, double explorationCoefficient);
    virtual ~UcbStepGeneratorFactory() = default;
    _NO_COPY_OR_MOVE(UcbStepGeneratorFactory);

    virtual std::unique_ptr<StepGenerator> createGenerator(SearchStatus &status,
            HistorySequence *sequence) override;
private:
    Solver *solver_;
    double explorationCoefficient_;
};

class UcbStepGenerator : public StepGenerator {
public:
    UcbStepGenerator(SearchStatus &status, HistorySequence *sequence,
            Solver *solver, double explorationCoefficient);
    ~UcbStepGenerator() = default;
    _NO_COPY_OR_MOVE(UcbStepGenerator);

    virtual Model::StepResult getStep() override;
private:
    HistorySequence *sequence_;
    Solver *solver_;
    Model *model_;
    double explorationCoefficient_;

    bool choseUnvisitedAction_;
};
} /* namespace solver */

#endif /* SOLVER_UCB_SEARCH_HPP_ */
