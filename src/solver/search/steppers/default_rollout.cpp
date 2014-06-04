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
        HistorySequence *sequence) {
    return std::make_unique<DefaultRolloutGenerator>(status, sequence, solver_, maxNSteps_);
}

/* ------------------------- DefaultRolloutGenerator ------------------------- */
DefaultRolloutGenerator::DefaultRolloutGenerator(SearchStatus &status, HistorySequence *sequence,
        Solver *solver, long maxNSteps) :
            StepGenerator(status),
            sequence_(sequence),
            solver_(solver),
            model_(solver_->getModel()),
            maxNSteps_(maxNSteps),
            currentNSteps_(0) {
}

Model::StepResult DefaultRolloutGenerator::getStep() {
    if (currentNSteps_ >= maxNSteps_) {
        // This stage is over.
        status_ = SearchStatus::STAGE_COMPLETE;
        return Model::StepResult { };
    }

    currentNSteps_++;
    HistoryEntry *lastEntry = sequence_->getLastEntry();
    HistoricalData *currentData = lastEntry->getAssociatedBeliefNode()->getHistoricalData();
    std::unique_ptr<Action> action = solver_->getActionPool()->getDefaultRolloutAction(currentData);

    // Generate the step and return it.
    return model_->generateStep(*lastEntry->getState(), *action);
}
} /* namespace solver */
