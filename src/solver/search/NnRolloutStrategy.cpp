#include "NnRolloutStrategy.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Solver.hpp"

#include "solver/mappings/ActionMapping.hpp"

namespace solver {

NnRolloutStrategy::NnRolloutStrategy(long maxNnComparisons,
        double maxNnDistance) :
        maxNnComparisons_(maxNnComparisons),
        maxNnDistance_(maxNnDistance) {
}

std::unique_ptr<SearchInstance> NnRolloutStrategy::createSearchInstance(
        Solver *solver, HistorySequence *sequence, long maximumDepth) {
    return std::make_unique<NnRolloutInstance>(maxNnComparisons_,
            maxNnDistance_, solver, sequence, maximumDepth);
}

NnRolloutInstance::NnRolloutInstance(long maxNnComparisons, double maxNnDistance,
        Solver *solver, HistorySequence *sequence, long maximumDepth) :
        AbstractSearchInstance(solver, sequence, maximumDepth),
        maxNnComparisons_(maxNnComparisons),
        maxNnDistance_(maxNnDistance),
        rootNeighborNode_(nullptr),
        currentNeighborNode_(nullptr),
        previousAction_(nullptr) {
}

SearchStatus NnRolloutInstance::initialize() {
    rootNeighborNode_ = solver_->getNNBelNode(currentNode_, maxNnDistance_,
                    maxNnComparisons_);
    currentNeighborNode_ = rootNeighborNode_;
    if (rootNeighborNode_ != nullptr) {
        status_ = SearchStatus::INITIAL;
    }
    return status_;
}

std::pair<SearchStatus, std::unique_ptr<Action>>
NnRolloutInstance::getStatusAndNextAction() {
    // If this is our first action, it *must* be one of the previously
    // unselected rollout actions.
    if (previousAction_ == nullptr) {
        previousAction_ = currentNode_->getMapping()->getRandomRolloutAction();
        return std::make_pair(SearchStatus::ROLLING_OUT, previousAction_->copy());
    }
    HistoryEntry *previousEntry = sequence_->getEntry(sequence_->getLength() - 2);
    currentNeighborNode_ = currentNeighborNode_->getChild(*previousAction_,
            *previousEntry->getObservation());

    if (currentNeighborNode_ == nullptr) {
        return std::make_pair(SearchStatus::ROLLOUT_COMPLETE, nullptr);
    }
    return std::make_pair(SearchStatus::ROLLING_OUT, currentNeighborNode_->getBestAction());
}

} /* namespace solver */
