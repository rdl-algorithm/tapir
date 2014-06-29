#include "solver/mappings/actions/ActionMappingEntry.hpp"

#include "solver/BeliefNode.hpp"

#include "solver/mappings/actions/ActionMapping.hpp"

namespace solver {
bool ActionMappingEntry::update(long deltaNVisits, double deltaTotalQ) {
    return updateValue(deltaNVisits, deltaTotalQ);
}
} /* namespace solver */
