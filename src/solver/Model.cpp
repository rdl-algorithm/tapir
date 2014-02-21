#include "Model.hpp"

#include "indexing/RTree.hpp"
#include "indexing/StateIndex.hpp"

namespace solver {
std::unique_ptr<StateIndex> Model::createStateIndex() {
    return std::make_unique<RTree>(getNStVars());
}
} /* namespace solver */
