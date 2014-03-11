#include "UcbSearchStrategy.hpp"

namespace solver {

UcbSearchStrategy::UcbSearchStrategy(double explorationCoefficient) :
        explorationCoefficient_(explorationCoefficient) {
}

std::unique_ptr<SearchInstance> UcbSearchStrategy::createSearchInstance(
       Solver *solver, HistorySequence *sequence) {
    return std::make_unique<UcbSearchInstance>(explorationCoefficient_,
            solver, sequence);
}

UcbSearchInstance::UcbSearchInstance(double explorationCoefficient,
        Solver *solver, HistorySequence *sequence) :
    SearchInstance(solver, sequence),
    explorationCoefficient_(explorationCoefficient) {
}

virtual std::pair<SearchStatus, std::unique_ptr<Action>>
UcbSearchInstance::getStatusAndNextAction() {
    double bestValue = -std::numeric_limits<double>::infinity();
    std::unique_ptr<Action> bestAction = nullptr;
    ActionMapping *mapping = currentNode_->getMapping();
    if (mapping->hasRolloutActions()) {
        return std::make_pair(SearchStatus::REACHED_ROLLOUT_NODE, nullptr);
    }
    for (ActionMappingEntry const *entry : currentNode_->getMapping()->getChildEntries()) {
        ActionNode *node = entry->getActionNode();
        double tmpValue = node->getMeanQValue() + (explorationCoefficient_ * std::sqrt(
                        std::log(currentNode_->getNParticles() / node->getNParticles())));
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
