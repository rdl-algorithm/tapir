#include "Model.hpp"

#include "RTree.hpp"
#include "StateIndex.hpp"

namespace solver {
std::unique_ptr<StateIndex> Model::createStateIndex() {
    return std::make_unique<RTree>(getNStVars());
}
} /* namespace solver */
