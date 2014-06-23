#include "solver/belief-estimators/estimators.hpp"

#include "solver/cached_values.hpp"
#include "solver/BeliefNode.hpp"

#include "solver/mappings/actions/ActionMapping.hpp"
#include "solver/mappings/actions/ActionMappingEntry.hpp"

namespace solver {
namespace estimators {
double average_q_value(BeliefNode const *node) {
    ActionMapping *mapping = node->getMapping();
    if (mapping->getNumberOfVisitedEntries() == 0) {
        return 0;
    }

    double totalQValue = 0;
    for (ActionMappingEntry const *entry : mapping->getVisitedEntries()) {
        totalQValue += entry->getTotalQValue();
    }

    double averageQValue = totalQValue / mapping->getTotalVisitCount();
    if (!std::isfinite(averageQValue)) {
        debug::show_message("Non-finite Q!");
    }
    return averageQValue;
}

double max_q_value(BeliefNode const *node) {
    ActionMapping *mapping = node->getMapping();
    if (mapping->getNumberOfVisitedEntries() == 0) {
        return 0;
    }

    double maxQValue = -std::numeric_limits<double>::infinity();
    for (ActionMappingEntry const *entry : mapping->getVisitedEntries()) {
        double qValue = entry->getMeanQValue();
        if (qValue > maxQValue) {
            maxQValue = qValue;
        }
    }
    return maxQValue;
}

double robust_q_value(BeliefNode const *node) {
    ActionMapping *mapping = node->getMapping();
    if (mapping->getNumberOfVisitedEntries() == 0) {
        return 0;
    }

    long maxVisitCount = 0;
    double robustQValue = -std::numeric_limits<double>::infinity();
    for (ActionMappingEntry const *entry : mapping->getVisitedEntries()) {
        double visitCount = entry->getVisitCount();
        double qValue = entry->getMeanQValue();
        if (visitCount > maxVisitCount) {
            maxVisitCount = visitCount;
            robustQValue = qValue;
        } else if (visitCount == maxVisitCount && qValue > robustQValue) {
            robustQValue = qValue;
        }
    }
    return robustQValue;
}
} /* namespace estimators */

EstimationFunction::EstimationFunction(std::function<double(BeliefNode const *)> function) :
            function_(function) {
}

void EstimationFunction::setQEstimator(Solver */*solver*/, BeliefNode *node) {
    std::unique_ptr<CachedValue<double>> cachedValue = std::make_unique<CachedValue<double>>(node,
            function_);
    node->setQEstimator(cachedValue.get());
    node->addCachedValue(std::move(cachedValue));
}
} /* namespace solver */
