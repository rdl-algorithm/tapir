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
/* ---------------------- LegalActionsBeliefData --------------------- */
LegalActionsBeliefData::LegalActionsBeliefData(RockSampleModel *model,
        GridPosition position) :
        model_(model),
        position_(position) {
}

std::unique_ptr<solver::BeliefData> LegalActionsBeliefData::createChildData(
        solver::Action const &action,
        solver::Observation const &/*observation*/) {
    RockSampleAction const &rsAction = static_cast<RockSampleAction const &>(action);

    bool isLegal;
    GridPosition nextPosition;
    std::tie(nextPosition, isLegal) = model_->makeNextPosition(position_, rsAction);
    if (!isLegal && model_->usingOnlyLegal()) {
        debug::show_message("ERROR: An illegal action was taken!?");
    }
    return std::make_unique<LegalActionsBeliefData>(model_, nextPosition);
}

/* ------------------------ LegalActionsModel ----------------------- */
std::unique_ptr<solver::ActionPool> LegalActionsModel::createActionPool(
        solver::Solver *solver) {
    return std::make_unique<LegalActionsPool>(solver, this, getNumberOfBins());
}

std::unique_ptr<solver::BeliefData> LegalActionsModel::createRootBeliefData() {
    RockSampleModel *model = dynamic_cast<RockSampleModel *>(this);
    return std::make_unique<LegalActionsBeliefData>(
            model, model->getStartPosition());
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
                    solver::DiscretizedActionMap(
                            observationPool, model, numberOfBins) {
}
void LegalActionsMap::initialize() {
    RockSampleModel &model = dynamic_cast<RockSampleModel &>(*model_);
    LegalActionsBeliefData const &data = (
            static_cast<LegalActionsBeliefData const &>(
                    *owningBeliefNode_->getBeliefData()));
    for (std::unique_ptr<solver::DiscretizedPoint> const &action : model.getAllActionsInOrder()) {
        RockSampleAction const &rsAction =
                static_cast<RockSampleAction const &>(*action);
        GridPosition nextPosition;
        bool isLegal;
        std::tie(nextPosition, isLegal) = model.makeNextPosition(data.position_, rsAction);
        if (isLegal || !model.usingOnlyLegal()) {
            addUnvisitedAction(rsAction.getBinNumber());
        }
    }
}

/* --------------------- LegalActionsTextSerializer -------------------- */
void LegalActionsTextSerializer::saveBeliefData(solver::BeliefData const *data,
        std::ostream &os) {
    os << std::endl;
    os << "CUSTOM DATA:" << std::endl;
    LegalActionsBeliefData const &legalData = (
                static_cast<LegalActionsBeliefData const &>(*data));
    os << "Position: " << legalData.position_ << std::endl;
    os << std::endl;
}
std::unique_ptr<solver::BeliefData> LegalActionsTextSerializer::loadBeliefData(
        std::istream &is) {

    std::string line;
    std::getline(is, line); // Blank line
    std::getline(is, line); // Header

    std::getline(is, line);
    std::string tmpStr;
    GridPosition position;
    std::istringstream(line) >> tmpStr >> position;

    std::getline(is, line); // Blank line

    RockSampleModel *model = dynamic_cast<RockSampleModel *>(model_);
    return std::make_unique<LegalActionsBeliefData>(model, position);
}
} /* namespace rocksample */
