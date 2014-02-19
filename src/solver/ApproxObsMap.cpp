#include "ApproxObsMap.hpp"

#include <limits>

#include "BeliefNode.hpp"
#include "Model.hpp"

namespace solver {
ApproxObsMap::ApproxObsMap(Model *model, double maxDistance) :
    model_(model),
    maxDistance_(maxDistance),
    obsNodes_() {
}

ApproxObsMap::~ApproxObsMap() {
}

BeliefNode *ApproxObsMap::getBelief(const Observation& obs) const {
    double shortestDistance = maxDistance_;
    BeliefNode *bestNode = nullptr;
    for (Entry const &entry : obsNodes_) {
        double distance = entry.first->distanceTo(obs);
        if (distance <= shortestDistance) {
            shortestDistance = distance;
            bestNode = entry.second.get();
        }
    }
    return bestNode;
}

BeliefNode *ApproxObsMap::createBelief(const Observation& obs) {
    Entry val = std::make_pair(obs.copy(), std::make_unique<BeliefNode>(
            model_->createActionMapping()));
    BeliefNode *node = val.second.get();
    obsNodes_.push_back(std::move(val));
    return node;
}

} /* namespace solver */
