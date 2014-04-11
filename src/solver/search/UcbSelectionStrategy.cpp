#include "UcbSelectionStrategy.hpp"

#include "solver/mappings/ActionMapping.hpp"

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"

namespace solver {

UcbSelectionStrategy::UcbSelectionStrategy(Solver *solver,
        double explorationCoefficient) :
        SearchStrategy(solver),
        explorationCoefficient_(explorationCoefficient) {
}

std::unique_ptr<SearchInstance> UcbSelectionStrategy::createSearchInstance(
        HistorySequence *sequence, long maximumDepth) {
    return std::make_unique<UcbSelectionInstance>(explorationCoefficient_,
            solver_, sequence, maximumDepth);
}

UcbSelectionInstance::UcbSelectionInstance(double explorationCoefficient,
        Solver *solver, HistorySequence *sequence, long maximumDepth) :
    AbstractSearchInstance(solver, sequence, maximumDepth),
    explorationCoefficient_(explorationCoefficient) {
}

std::pair<SearchStatus, std::unique_ptr<Action>> UcbSelectionInstance::getStatusAndNextAction() {
    ActionMapping *mapping = currentNode_->getMapping();
    if (mapping->hasUnvisitedActions()) {
        return std::make_pair(SearchStatus::REACHED_ROLLOUT_NODE, nullptr);
    }

    double bestValue = -std::numeric_limits<double>::infinity();
    std::unique_ptr<Action> bestAction = nullptr;
    for (ActionMappingEntry const *entry : mapping->getVisitedEntries()) {
        double tmpValue = entry->getMeanQValue() + explorationCoefficient_ * std::sqrt(
                        std::log(mapping->getTotalVisitCount()) / entry->getVisitCount());
        if (!std::isfinite(tmpValue)) {
            debug::show_message("ERROR: Infinite/NaN value!?");
        }
        if (bestValue < tmpValue) {
            bestValue = tmpValue;
            bestAction = entry->getAction();
        }
    }
    if (bestAction == nullptr) {
        debug::show_message("ERROR: node has no actions!?");
        return std::make_pair(SearchStatus::ERROR, nullptr);
    }
    return std::make_pair(SearchStatus::INSIDE_TREE, std::move(bestAction));
}

} /* namespace solver */
