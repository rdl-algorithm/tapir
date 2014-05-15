#include "average_q_max_rec.hpp"

namespace solver {
std::unique_ptr<BeliefQValueEstimator> AverageQMaxChildStrategy::createEstimator(
        ActionMapping *mapping) {
    return std::make_unique<AverageQMaxChild>(mapping);
}

AverageQMaxChild::AverageQMaxChild(ActionMapping *mapping) :
            mapping_(mapping),
            beliefQValue_(-std::numeric_limits<double>::infinity()),
            recommendedAction_(nullptr) {
}

void AverageQMaxChild::recalculate() {
    if (mapping_->getTotalVisitCount() <= 0) {
        beliefQValue_ = -std::numeric_limits<double>::infinity();
        recommendedAction_ = nullptr;
        return;
    }

    double bestQValue = -std::numeric_limits<double>::infinity();
    recommendedAction_ = nullptr;
    double totalQValue = 0;
    for (ActionMappingEntry const *entry : mapping_->getVisitedEntries()) {
        totalQValue += entry->getTotalQValue();

        double qValue = entry->getMeanQValue();
        if (qValue > bestQValue) {
            bestQValue = qValue;
            recommendedAction_ = entry->getAction();
        }
    }

    beliefQValue_ = totalQValue / mapping_->getTotalVisitCount();
}

double AverageQMaxChild::getBeliefQValue() const {
    return beliefQValue_;
}

std::unique_ptr<Action> AverageQMaxChild::getRecommendedAction() const {
    if (recommendedAction_ != nullptr) {
        return recommendedAction_->copy();
    }
    return mapping_->getRandomUnvisitedAction();

}

} /* namespace solver */
