#include "max_q_max_rec.hpp"

namespace solver {
std::unique_ptr<BeliefQValueEstimator> MaxQMaxChildStrategy::createEstimator(
        ActionMapping *mapping) {
    return std::make_unique<MaxQMaxChild>(mapping);
}

MaxQMaxChild::MaxQMaxChild(ActionMapping *mapping) :
            mapping_(mapping),
            beliefQValue_(-std::numeric_limits<double>::infinity()),
            recommendedAction_(nullptr) {
}

void MaxQMaxChild::recalculate() {
    if (mapping_->getTotalVisitCount() <= 0) {
        beliefQValue_ = -std::numeric_limits<double>::infinity();
        recommendedAction_ = nullptr;
        return;
    }

    beliefQValue_ = -std::numeric_limits<double>::infinity();
    recommendedAction_ = nullptr;
    for (ActionMappingEntry const *entry: mapping_->getVisitedEntries()) {
        double qValue = entry->getMeanQValue();
        if (qValue > beliefQValue_) {
            beliefQValue_ = qValue;
            recommendedAction_ = entry->getAction();
        }
    }
}

double MaxQMaxChild::getBeliefQValue() const {
    return beliefQValue_;
}

std::unique_ptr<Action> MaxQMaxChild::getRecommendedAction() const {
    if (recommendedAction_ != nullptr) {
        return recommendedAction_->copy();
    }
    return mapping_->getRandomUnvisitedAction();

}

} /* namespace solver */
