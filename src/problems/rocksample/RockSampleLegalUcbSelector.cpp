#include "RockSampleLegalUcbSelector.hpp"

#include "solver/mappings/ActionMapping.hpp"
#include "solver/mappings/discretized_actions.hpp"

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"

#include "RockSampleAction.hpp"
#include "RockSampleModel.hpp"
#include "RockSampleState.hpp"

#include "problems/shared/parsers.hpp"

namespace rocksample {
RockSampleLegalUcbSelector::RockSampleLegalUcbSelector(solver::Solver *solver,
        double explorationCoefficient) :
        solver::SearchStrategy(solver),
        explorationCoefficient_(explorationCoefficient) {
}

std::unique_ptr<solver::SearchInstance> RockSampleLegalUcbSelector::createSearchInstance(
        solver::HistorySequence *sequence, long maximumDepth) {
    return std::make_unique<RockSampleLegalUcbSelectorInstance>(explorationCoefficient_,
            solver_, sequence, maximumDepth);
}

RockSampleLegalUcbSelectorInstance::RockSampleLegalUcbSelectorInstance(
        double explorationCoefficient,
        solver::Solver *solver,
        solver::HistorySequence *sequence,
        long maximumDepth) :
        solver::AbstractSearchInstance(solver, sequence, maximumDepth),
    explorationCoefficient_(explorationCoefficient) {
}

std::pair<solver::SearchStatus, std::unique_ptr<solver::Action>> RockSampleLegalUcbSelectorInstance::getStatusAndNextAction() {
    double bestValue = -std::numeric_limits<double>::infinity();
    std::unique_ptr<solver::Action> bestAction = nullptr;
    solver::ActionMapping *mapping = currentNode_->getMapping();

    // If it's a newly created mapping, we remove any illegal actions from it.
    if (mapping->getNChildren() == 0) {
        RockSampleModel &model = dynamic_cast<RockSampleModel &>(*model_);
        RockSampleState const &state = static_cast<RockSampleState const &>(
                    *sequence_->getLastEntry()->getState());
        solver::DiscretizedActionMap &map = static_cast<solver::DiscretizedActionMap &>(*mapping);
        for (std::unique_ptr<solver::Action> &action : mapping->getUnvisitedActions()) {
            RockSampleAction const &rsAction = static_cast<RockSampleAction const &>(*action);
            // Delete any illegal actions
            if (!model.makeNextState(state, rsAction).second) {
                map.deleteUnvisitedAction(rsAction.getBinNumber());
            }
        }
    }

    // If there are legal actions we haven't tried, we go to rollout mode now.
    if (mapping->hasUnvisitedActions()) {
        return std::make_pair(solver::SearchStatus::REACHED_ROLLOUT_NODE, nullptr);
    }

    // We've tried all of the legal actions, so now we use UCB.
    for (solver::ActionMappingEntry const *entry : mapping->getChildEntries()) {
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
        return std::make_pair(solver::SearchStatus::ERROR, nullptr);
    }
    return std::make_pair(solver::SearchStatus::INSIDE_TREE, std::move(bestAction));
}

std::unique_ptr<solver::SearchStrategy> RockSampleLegalUcbSelectorParser::parse(
            solver::Solver *solver,
            ParserSet<solver::SearchStrategy> */*allParser*/,
            std::vector<std::string> args) {
    double explorationCoefficient;
    std::istringstream(args[0]) >> explorationCoefficient;
    return std::make_unique<RockSampleLegalUcbSelector>(solver,
            explorationCoefficient);
}

} /* namespace solver */
