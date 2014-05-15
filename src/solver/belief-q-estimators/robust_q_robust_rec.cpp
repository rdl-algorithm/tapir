#include "robust_q_robust_rec.hpp"

namespace solver {
std::unique_ptr<BeliefQValueEstimator> RobustQRobustChildStrategy::createEstimator(
        ActionMapping *mapping) {
    return std::make_unique<RobustQRobustChild>(mapping);
}

RobustQRobustChild::RobustQRobustChild(ActionMapping *mapping) :
            mapping_(mapping),
            beliefQValue_(-std::numeric_limits<double>::infinity()),
            recommendedAction_(nullptr) {
}

void RobustQRobustChild::recalculate() {
    if (mapping_->getTotalVisitCount() <= 0) {
        beliefQValue_ = -std::numeric_limits<double>::infinity();
        recommendedAction_ = nullptr;
        return;
    }

    long bestVisitCount = 0;
    beliefQValue_ = -std::numeric_limits<double>::infinity();
    recommendedAction_ = nullptr;
    for (ActionMappingEntry const *entry : mapping_->getVisitedEntries()) {
        long visitCount = entry->getVisitCount();
        double qValue = entry->getMeanQValue();
        if (visitCount > bestVisitCount
                || (visitCount == bestVisitCount && qValue > beliefQValue_)) {
            bestVisitCount = visitCount;
            beliefQValue_ = qValue;
            recommendedAction_ = entry->getAction();
        }
    }
}

double RobustQRobustChild::getBeliefQValue() const {
    return beliefQValue_;
}

std::unique_ptr<Action> RobustQRobustChild::getRecommendedAction() const {
    if (recommendedAction_ != nullptr) {
        return recommendedAction_->copy();
    }
    return mapping_->getRandomUnvisitedAction();

}

} /* namespace solver */
