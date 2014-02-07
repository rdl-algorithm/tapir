#include "SimpleObsMap.hpp"

#include "BeliefNode.hpp"

namespace solver {
SimpleObsMap::SimpleObsMap() :
    obsNodes_() {
}

SimpleObsMap::~SimpleObsMap() {
}

BeliefNode* SimpleObsMap::getBelief(const Observation& obs) {
    try {
        return obsNodes_.at(obs.copy()).get();
    } catch (const std::out_of_range &oor) {
        return nullptr;
    }
}

BeliefNode* SimpleObsMap::createBelief(const Observation& obs) {
    auto val = std::make_pair(obs.copy(), std::make_unique<BeliefNode>());
    BeliefNode *node = val.second.get();
    obsNodes_.insert(std::move(val));
    return node;
}

} /* namespace solver */
