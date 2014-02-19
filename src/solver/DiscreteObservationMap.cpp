#include "DiscreteObservationMap.hpp"

#include "BeliefNode.hpp"
#include "Model.hpp"

namespace solver {
DiscreteObservationMap::DiscreteObservationMap(Model *model) :
    model_(model),
    observations_(),
    obsNodes_() {
}

DiscreteObservationMap::~DiscreteObservationMap() {
}

BeliefNode* DiscreteObservationMap::getBelief(Observation const &obs) const {
    try {
        return obsNodes_.at(&obs).get();
    } catch (const std::out_of_range &oor) {
        return nullptr;
    }
}

BeliefNode* DiscreteObservationMap::createBelief(const Observation& obs) {
    std::unique_ptr<Observation> observation(obs.copy());
    std::unique_ptr<BeliefNode> node(std::make_unique<BeliefNode>(
            model_->createActionMapping()));
    BeliefNode *nodePtr = node.get();
    obsNodes_.emplace(observation.get(), std::move(node));
    observations_.push_back(std::move(observation));
    return nodePtr;
}

} /* namespace solver */
