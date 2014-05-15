#include "UcbSelectionStrategy.hpp"

#include "solver/mappings/actions/ActionMapping.hpp"

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
            getSolver(), sequence, maximumDepth);
}

UcbSelectionInstance::UcbSelectionInstance(double explorationCoefficient,
        Solver *solver, HistorySequence *sequence, long maximumDepth) :
        AbstractSelectionInstance(solver, sequence, maximumDepth),
    choseUnvisitedAction_(false),
    explorationCoefficient_(explorationCoefficient) {
}

SearchStep UcbSelectionInstance::getSearchStep(BeliefNode *currentNode) {
    if (choseUnvisitedAction_) {
        // We've reached the new leaf - this (UCB) search is over.
        return SearchStep {SearchStatus::REACHED_ROLLOUT_NODE, nullptr, false};
    }

    ActionMapping *mapping = currentNode->getMapping();
    // If there are unvisited actions, we take one, and we're finished with UCB.
    if (mapping->hasUnvisitedActions()) {
        choseUnvisitedAction_ = true;
        return SearchStep {SearchStatus::INSIDE_TREE, mapping->getRandomUnvisitedAction(), true};
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
        return SearchStep {SearchStatus::ERROR, nullptr, false};
    }
    return SearchStep {SearchStatus::INSIDE_TREE, std::move(bestAction), true};
}

} /* namespace solver */
