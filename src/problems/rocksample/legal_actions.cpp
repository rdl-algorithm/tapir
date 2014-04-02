#include "legal_actions.hpp"

#include <iostream>
#include <sstream>

#include "RockSampleAction.hpp"
#include "RockSampleModel.hpp"

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"

#include "solver/abstract-problem/Action.hpp"

#include "solver/mappings/ActionMapping.hpp"
#include "solver/mappings/ObservationMapping.hpp"

namespace rocksample {
/* ------------------------ LegalActionsModel ----------------------- */
std::unique_ptr<solver::ActionPool> LegalActionsModel::createActionPool(
        solver::Solver *solver) {
    return std::make_unique<LegalActionsPool>(solver, this, getNumberOfBins());
}

/* ------------------------ LegalActionsPool ----------------------- */
LegalActionsPool::LegalActionsPool(solver::Solver *solver,
        solver::ModelWithDiscretizedActions *model, long numberOfBins) :
                solver::DiscretizedActionPool(solver, model, numberOfBins) {
}
std::unique_ptr<solver::ActionMapping> LegalActionsPool::createActionMapping() {
    return std::make_unique<LegalActionsMap>(
            getSolver()->getObservationPool(), model_, numberOfBins_);
}

/* ------------------------- LegalActionsMap ------------------------ */
LegalActionsMap::LegalActionsMap(solver::ObservationPool *observationPool,
            solver::ModelWithDiscretizedActions *model, long numberOfBins) :
                    solver::DiscretizedActionMap(observationPool, model, numberOfBins),
                    position_() {
}
void LegalActionsMap::initialize() {
    solver::ObservationMappingEntry *obsEntry = owningBeliefNode_->getParentEntry();
    RockSampleModel &model = dynamic_cast<RockSampleModel &>(*model_);
    if (obsEntry == nullptr) {
        position_ = model.getStartPosition();
    } else {
        solver::ActionMappingEntry *actEntry = obsEntry->getMapping()->getOwner()->getParentEntry();
        std::unique_ptr<solver::Action> action = actEntry->getAction();
        RockSampleAction const &rsAction = static_cast<RockSampleAction const &>(*action);
        LegalActionsMap *parentMap = static_cast<LegalActionsMap *>(actEntry->getMapping());
        bool isLegal;
        std::tie(position_, isLegal) = model.makeNextPosition(parentMap->position_, rsAction);
        if (!isLegal && model.usingOnlyLegal()) {
            debug::show_message("An illegal action was taken!?");
        }
    }
    for (std::unique_ptr<solver::DiscretizedPoint> const &action : model.getAllActionsInOrder()) {
        RockSampleAction const &rsAction =
                static_cast<RockSampleAction const &>(*action);
        GridPosition nextPosition;
        bool isLegal;
        std::tie(nextPosition, isLegal) = model.makeNextPosition(position_, rsAction);
        if (isLegal || !model.usingOnlyLegal()) {
            addUnvisitedAction(rsAction.getBinNumber());
        }
    }
}

/* --------------------- LegalActionsTextSerializer -------------------- */
void LegalActionsTextSerializer::saveCustomMappingData(
        solver::DiscretizedActionMap const &map, std::ostream &os) {
    LegalActionsMap const &legalMap = static_cast<LegalActionsMap const &>(map);
    os << std::endl;
    os << "CUSTOM DATA:" << std::endl;
    os << "Position: " << legalMap.position_ << std::endl;
    os << std::endl; // Blank line
}
void LegalActionsTextSerializer::loadCustomMappingData(
        solver::DiscretizedActionMap &map, std::istream &is) {
    LegalActionsMap &legalMap = static_cast<LegalActionsMap &>(map);
    std::string line;

    std::getline(is, line); // Blank line
    std::getline(is, line); // Header

    std::getline(is, line);
    std::string tmpStr;
    std::istringstream(line) >> tmpStr >> legalMap.position_;

    std::getline(is, line); // Blank line
}

} /* namespace rocksample */
