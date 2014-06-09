#include "solver/search/steppers/default_rollout.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Solver.hpp"

#include "solver/abstract-problem/Model.hpp"

#include "solver/mappings/actions/ActionPool.hpp"
#include "solver/mappings/actions/ActionMapping.hpp"

namespace solver {
/* ------------------------- DefaultRolloutFactory ------------------------- */
DefaultRolloutFactory::DefaultRolloutFactory(Solver *solver, long maxNSteps) :
            solver_(solver),
            maxNSteps_(maxNSteps) {
}

std::unique_ptr<StepGenerator> DefaultRolloutFactory::createGenerator(SearchStatus &status,
        HistoryEntry const */*entry*/, State const */*state*/, HistoricalData const */*data*/) {
    return std::make_unique<DefaultRolloutGenerator>(status, solver_, maxNSteps_);
}

/* ------------------------- DefaultRolloutGenerator ------------------------- */
DefaultRolloutGenerator::DefaultRolloutGenerator(SearchStatus &status,
        Solver *solver, long maxNSteps) :
            StepGenerator(status),
            model_(solver->getModel()),
            maxNSteps_(maxNSteps),
            currentNSteps_(0) {
    status_ = SearchStatus::INITIAL;
}

Model::StepResult DefaultRolloutGenerator::getStep(HistoryEntry const */*entry*/, State const *state,
        HistoricalData const *data) {
    if (currentNSteps_ >= maxNSteps_) {
        // This stage is over.
        status_ = SearchStatus::OUT_OF_STEPS;
        return Model::StepResult { };
    }

    currentNSteps_++;
    std::unique_ptr<Action> action = model_->getRolloutAction(data, state);

    // Generate the step and return it.
    return model_->generateStep(*state, *action);
}
} /* namespace solver */
