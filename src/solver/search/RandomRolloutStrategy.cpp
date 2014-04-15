#include "RandomRolloutStrategy.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/Solver.hpp"

#include "solver/abstract-problem/Model.hpp"

#include "solver/mappings/ActionPool.hpp"
#include "solver/mappings/ActionMapping.hpp"

namespace solver {

RandomRolloutStrategy::RandomRolloutStrategy(Solver *solver, long maxNSteps) :
        SearchStrategy(solver),
        maxNSteps_(maxNSteps) {
}

std::unique_ptr<SearchInstance> RandomRolloutStrategy::createSearchInstance(
        HistorySequence *sequence, long maximumDepth) {
    return std::make_unique<RandomRolloutInstance>(
            maxNSteps_, solver_, sequence, maximumDepth);
}

RandomRolloutInstance::RandomRolloutInstance(long maxNSteps, Solver *solver,
        HistorySequence *sequence, long maximumDepth) :
        AbstractRolloutInstance(solver, sequence, maximumDepth),
                maxNSteps_(maxNSteps),
                currentNSteps_(0) {
}

SearchStep RandomRolloutInstance::getSearchStep(Solver *solver, HistorySequence */*sequence*/,
        HistoricalData *currentData) {
    if (currentNSteps_ >= maxNSteps_) {
        return {SearchStatus::ROLLOUT_COMPLETE, nullptr, false};
    }
    currentNSteps_++;
    std::unique_ptr<Action> action = solver->getActionPool()->getRolloutAction(currentData);
    return {SearchStatus::ROLLING_OUT, std::move(action), false};
}

} /* namespace solver */
