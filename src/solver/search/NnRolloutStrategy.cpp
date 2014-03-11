#include "NnRolloutStrategy.hpp"

namespace solver {

NnRolloutStrategy::NnRolloutStrategy(long maxNnComparisons,
        double maxNnDistance) :
        maxNnComparisons_(maxNnComparisons),
        maxNnDistance_(maxNnDistance) {
}

std::unique_ptr<SearchInstance> NnRolloutStrategy::createSearchInstance(
        Solver *solver, HistorySequence *sequence) {
    return std::make_unique<NnRolloutInstance>(maxNnComparisons_,
            maxNnDistance_, solver, sequence);
}

NnRolloutInstance::NnRolloutInstance(long maxNnComparisons, double maxNnDistance,
        Solver *solver, HistorySequence *sequence) :
        SearchInstance(solver, sequence),
        rootNeighborNode_(solver_->getNNBelNode(currentNode_, maxNnDistance,
                maxNnComparisons)),
        currentNeighborNode_(rootNeighborNode_),
        previousAction_(nullptr) {
}

std::pair<SearchStatus, std::unique_ptr<Action>>
NnRolloutInstance::getStatusAndNextAction() {
    if (rootNeighborNode_ == nullptr) {
        return std::make_pair(SearchStatus::ERROR, nullptr);
    }
    // If this is our first action, it *must* be one of the previously
    // unselected rollout actions.
    if (previousAction_ == nullptr) {
        std::vector<std::unique_ptr<Action>> rolloutActions = currentNode_->getMapping()->getRolloutActions();
        long randomIndex = std::uniform_int_distribution<long>(
                0, rolloutActions.size()-1)(model_->getRandomGenerator());
        previousAction_ = std::move(rolloutActions[randomIndex]);
        return std::make_pair(SearchStatus::ROLLING_OUT, previousAction_->copy());
    }
    HistoryEntry *previousEntry = sequence_->getEntry(sequence_->getLength() - 2);
    currentNeighborNode_ = currentNeighborNode_->getChild(*previousAction_,
            *previousEntry->observation_);

    if (currentNeighborNode_ == nullptr) {
        return std::make_pair(SearchStatus::ROLLOUT_COMPLETE, nullptr);
    }
    return std::make_pair(SearchStatus::ROLLING_OUT, currentNeighborNode_->getBestAction());
}

} /* namespace solver */
