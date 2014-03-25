#include "RockSampleLegalUcbSelector.hpp"

#include "solver/mappings/ActionMapping.hpp"

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"

namespace solver {

RockSampleLegalUcbSelector::RockSampleLegalUcbSelector(Solver *solver,
        double explorationCoefficient) :
        SearchStrategy(solver),
        explorationCoefficient_(explorationCoefficient) {
}

std::unique_ptr<SearchInstance> RockSampleLegalUcbSelector::createSearchInstance(
        HistorySequence *sequence, long maximumDepth) {
    return std::make_unique<RockSampleLegalUcbSelectorInstance>(explorationCoefficient_,
            solver_, sequence, maximumDepth);
}

RockSampleLegalUcbSelectorInstance::RockSampleLegalUcbSelectorInstance(double explorationCoefficient,
        Solver *solver, HistorySequence *sequence, long maximumDepth) :
    AbstractSearchInstance(solver, sequence, maximumDepth),
    explorationCoefficient_(explorationCoefficient) {
}

std::pair<SearchStatus, std::unique_ptr<Action>> RockSampleLegalUcbSelectorInstance::getStatusAndNextAction() {
    double bestValue = -std::numeric_limits<double>::infinity();
    std::unique_ptr<Action> bestAction = nullptr;
    ActionMapping *mapping = currentNode_->getMapping();
    if (mapping->hasUnvisitedActions()) {
        return std::make_pair(SearchStatus::REACHED_ROLLOUT_NODE, nullptr);
    }
    for (ActionMappingEntry const *entry : mapping->getChildEntries()) {
        double tmpValue = entry->getMeanQValue() + (explorationCoefficient_ * std::sqrt(
                        std::log((double)mapping->getTotalVisitCount() / entry->getVisitCount())));
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
