#include "RandomRolloutStrategy.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/abstract-problem/Model.hpp"
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
                AbstractSearchInstance(solver, sequence, maximumDepth),
                maxNSteps_(maxNSteps),
                currentNSteps_(0) {
}

std::pair<SearchStatus, std::unique_ptr<Action>>
RandomRolloutInstance::getStatusAndNextAction() {
    if (currentNSteps_ >= maxNSteps_) {
        return std::make_pair(SearchStatus::ROLLOUT_COMPLETE, nullptr);
    }
    currentNSteps_++;
    std::unique_ptr<Action> action = currentNode_->getMapping()->getRandomRolloutAction();
    return std::make_pair(SearchStatus::ROLLING_OUT, std::move(action));
}

} /* namespace solver */
