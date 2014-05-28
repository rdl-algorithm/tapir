#include "solver/action-choosers/choosers.hpp"

#include "solver/BeliefNode.hpp"

#include "solver/mappings/actions/ActionMapping.hpp"
#include "solver/mappings/actions/ActionMappingEntry.hpp"

namespace solver {
namespace choosers {
std::unique_ptr<Action> max_action(BeliefNode const *node) {
    std::unique_ptr<Action> maxAction = nullptr;
    double maxQValue = -std::numeric_limits<double>::infinity();

    ActionMapping *mapping = node->getMapping();
    for (ActionMappingEntry const *entry : mapping->getVisitedEntries()) {
        double qValue = entry->getMeanQValue();
        if (qValue > maxQValue) {
            maxQValue = qValue;
            maxAction = entry->getAction();
        }
    }
    return std::move(maxAction);
}

std::unique_ptr<Action> robust_action(BeliefNode const *node) {
    std::unique_ptr<Action> robustAction = nullptr;
    long maxVisitCount = 0;
    double robustQValue = -std::numeric_limits<double>::infinity();

    ActionMapping *mapping = node->getMapping();
    for (ActionMappingEntry const *entry : mapping->getVisitedEntries()) {
        double visitCount = entry->getVisitCount();
        double qValue = entry->getMeanQValue();
        if (visitCount > maxVisitCount) {
            maxVisitCount = visitCount;
            robustQValue = qValue;
            robustAction = entry->getAction();
        } else if (visitCount == maxVisitCount && qValue > robustQValue) {
            robustQValue = qValue;
            robustAction = entry->getAction();
        }
    }
    return std::move(robustAction);
}

std::unique_ptr<Action> ucb_action(BeliefNode const *node, double explorationCoefficient) {
    std::unique_ptr<Action> ucbAction = nullptr;
    double maxUcbValue = -std::numeric_limits<double>::infinity();

    ActionMapping *mapping = node->getMapping();
    for (ActionMappingEntry const *entry : mapping->getVisitedEntries()) {
        double tmpValue = entry->getMeanQValue() + explorationCoefficient * std::sqrt(
                                std::log(mapping->getTotalVisitCount()) / entry->getVisitCount());
        if (!std::isfinite(tmpValue)) {
            debug::show_message("ERROR: Infinite/NaN value!?");
        }
        if (maxUcbValue < tmpValue) {
            maxUcbValue = tmpValue;
            ucbAction = entry->getAction();
        }
    }
    return std::move(ucbAction);
}
} /* namespace choosers */

ActionChoosingFunction::ActionChoosingFunction(
        std::function<std::unique_ptr<Action>(BeliefNode const *)> function) :
        function_(function) {
}

void ActionChoosingFunction::setActionChooser(Solver */*solver*/, BeliefNode *node) {
    node->setActionChooser(function_);
}
} /* namespace solver */
