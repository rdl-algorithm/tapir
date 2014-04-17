#include "DefaultRolloutStrategy.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/Solver.hpp"

#include "solver/abstract-problem/Model.hpp"

#include "solver/mappings/ActionPool.hpp"
#include "solver/mappings/ActionMapping.hpp"

namespace solver {

DefaultRolloutStrategy::DefaultRolloutStrategy(Solver *solver, long maxNSteps) :
        SearchStrategy(solver),
        maxNSteps_(maxNSteps) {
}

std::unique_ptr<SearchInstance> DefaultRolloutStrategy::createSearchInstance(
        HistorySequence *sequence, long maximumDepth) {
    return std::make_unique<DefaultRolloutInstance>(
            maxNSteps_, getSolver(), sequence, maximumDepth);
}

DefaultRolloutInstance::DefaultRolloutInstance(long maxNSteps, Solver *solver,
        HistorySequence *sequence, long maximumDepth) :
        AbstractRolloutInstance(solver, sequence, maximumDepth),
                maxNSteps_(maxNSteps),
                currentNSteps_(0) {
}

SearchStep DefaultRolloutInstance::getSearchStep(HistoricalData *currentData) {
    if (currentNSteps_ >= maxNSteps_) {
        return SearchStep {SearchStatus::ROLLOUT_COMPLETE, nullptr, false};
    }
    currentNSteps_++;
    std::unique_ptr<Action> action = getSolver()->getActionPool()->getDefaultRolloutAction(currentData);
    return SearchStep {SearchStatus::ROLLING_OUT, std::move(action), false};
}

} /* namespace solver */
