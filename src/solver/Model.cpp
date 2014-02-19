#include "Model.hpp"

#include "RTree.hpp"
#include "DiscreteObservationMap.hpp"
#include "StateIndex.hpp"

namespace solver {
Model::Model(RandomGenerator *randGen) : randGen_(randGen) {
}

std::unique_ptr<StateIndex> Model::createStateIndex() {
    return std::make_unique<RTree>(getNStVars());
}

std::unique_ptr<ObservationMapping> Model::createObservationMapping() {
    return std::make_unique<DiscreteObservationMap>(this);
}
} /* namespace solver */
