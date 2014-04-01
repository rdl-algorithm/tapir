#include "legal_actions.hpp"

namespace rocksample {
/* ------------------------ LegalActionsModel ----------------------- */
std::unique_ptr<solver::ActionPool> LegalActionsModel::createActionPool() {
    return std::make_unique<LegalActionsPool>(this, getNumberOfBins());
}

/* ------------------------ LegalActionsPool ----------------------- */
LegalActionsPool::LegalActionsPool(solver::ModelWithDiscretizedActions *model,
        long numberOfBins) :
                solver::DiscretizedActionPool(model, numberOfBins) {
}
std::unique_ptr<solver::ActionMapping> LegalActionsPool::createActionMapping() {
    return std::make_unique<LegalActionsMap>(observationPool_, model_, numberOfBins_);
}

/* ------------------------- LegalActionsMap ------------------------ */
void LegalActionsMap::initialize(solver::BeliefNode *node) {
    // debug::show_message("Creating LAM!");
    DiscretizedActionMap::initialize(node);
}
LegalActionsMap::LegalActionsMap(solver::ObservationPool *observationPool,
            solver::ModelWithDiscretizedActions *model, long numberOfBins) :
                    solver::DiscretizedActionMap(observationPool, model, numberOfBins) {
}

/* --------------------- LegalActionsTextSerializer -------------------- */
void LegalActionsTextSerializer::saveCustomMappingData(
        solver::DiscretizedActionMap const &/*map*/, std::ostream &/*os*/) {
}
void LegalActionsTextSerializer::loadCustomMappingData(
        solver::DiscretizedActionMap &/*map*/, std::istream &/*is*/) {
}

} /* namespace rocksample */
