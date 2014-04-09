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
/* ---------------------- PositionData --------------------- */
PositionData::PositionData(RockSampleModel *model,
        GridPosition position) :
        model_(model),
        position_(position) {
}

std::unique_ptr<solver::HistoricalData> PositionData::createChild(
        solver::Action const &action,
        solver::Observation const &/*observation*/) {
    RockSampleAction const &rsAction = static_cast<RockSampleAction const &>(action);

    bool isLegal;
    GridPosition nextPosition;
    std::tie(nextPosition, isLegal) = model_->makeNextPosition(position_, rsAction);
    if (!isLegal && model_->usingOnlyLegal()) {
        debug::show_message("ERROR: An illegal action was taken!?");
    }
    return std::make_unique<PositionData>(model_, nextPosition);
}

void PositionData::print(std::ostream &os) const {
    os << "Position: " << position_ << std::endl;
}

/* ------------------------ LegalActionsModel ----------------------- */
std::unique_ptr<solver::ActionPool> LegalActionsModel::createActionPool(
        solver::Solver *solver) {
    return std::make_unique<LegalActionsPool>(solver, this, getNumberOfBins());
}

std::unique_ptr<solver::HistoricalData> LegalActionsModel::createRootInfo() {
    RockSampleModel *model = dynamic_cast<RockSampleModel *>(this);
    return std::make_unique<PositionData>(
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
    PositionData const &data = (
            static_cast<PositionData const &>(
                    *owningBeliefNode_->getHistoricalData()));
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
void LegalActionsTextSerializer::saveHistoricalData(solver::HistoricalData const *data,
        std::ostream &os) {
    os << std::endl;
    os << "CUSTOM DATA:" << std::endl;
    PositionData const &legalData = (
                static_cast<PositionData const &>(*data));
    os << "Position: " << legalData.position_ << std::endl;
    os << std::endl;
}
std::unique_ptr<solver::HistoricalData> LegalActionsTextSerializer::loadHistoricalData(
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
    return std::make_unique<PositionData>(model, position);
}
} /* namespace rocksample */
