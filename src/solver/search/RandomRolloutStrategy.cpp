#include "RandomRolloutStrategy.hpp"

namespace solver {

std::unique_ptr<SearchInstance> RandomRolloutStrategy::createSearchInstance(
       Solver *solver, HistorySequence *sequence) {
    return std::make_unique<RandomRolloutInstance>(solver, sequence);
}

RandomRolloutInstance::RandomRolloutInstance(long maxNSteps, Solver *solver,
        HistorySequence *sequence) :
                SearchInstance(solver, sequence),
                maxNSteps_(maxNSteps),
                currentNSteps_(0) {
}

std::pair<SearchStatus, std::unique_ptr<Action>>
RandomRolloutInstance::getStatusAndNextAction() {
    if (currentNSteps_ >= maxNSteps_) {
        return std::make_pair(SearchStatus::ROLLOUT_COMPLETE, nullptr);
    }
    currentNSteps_++;
    std::vector<std::unique_ptr<Action>> rolloutActions = currentNode_->getMapping()->getRolloutActions();
    long randomIndex = std::uniform_int_distribution<long>(
            0, rolloutActions.size()-1)(model_->getRandomGenerator());
    return std::make_pair(SearchStatus::ROLLING_OUT, std::move(rolloutActions[randomIndex]));
}

} /* namespace solver */
