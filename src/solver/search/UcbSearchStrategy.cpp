#include "UcbSearchStrategy.hpp"

#include "solver/mappings/ActionMapping.hpp"

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"

namespace solver {

UcbSearchStrategy::UcbSearchStrategy(double explorationCoefficient) :
        explorationCoefficient_(explorationCoefficient) {
}

std::unique_ptr<SearchInstance> UcbSearchStrategy::createSearchInstance(
       Solver *solver, HistorySequence *sequence, long maximumDepth) {
    return std::make_unique<UcbSearchInstance>(explorationCoefficient_,
            solver, sequence, maximumDepth);
}

UcbSearchInstance::UcbSearchInstance(double explorationCoefficient,
        Solver *solver, HistorySequence *sequence, long maximumDepth) :
    AbstractSearchInstance(solver, sequence, maximumDepth),
    explorationCoefficient_(explorationCoefficient) {
}

std::pair<SearchStatus, std::unique_ptr<Action>> UcbSearchInstance::getStatusAndNextAction() {
    double bestValue = -std::numeric_limits<double>::infinity();
    std::unique_ptr<Action> bestAction = nullptr;
    ActionMapping *mapping = currentNode_->getMapping();
    if (mapping->hasRolloutActions()) {
        return std::make_pair(SearchStatus::REACHED_ROLLOUT_NODE, nullptr);
    }
    for (ActionMappingEntry const *entry : currentNode_->getMapping()->getChildEntries()) {
        ActionNode *node = entry->getActionNode();
        double tmpValue = node->getQValue() + (explorationCoefficient_ * std::sqrt(
                        std::log(currentNode_->getNParticles() / node->getNParticles())));
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
