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

std::vector<long> PositionData::generateLegalActions() const {
    std::vector<long> legalActions;
    for (std::unique_ptr<solver::DiscretizedPoint> const &action : model_->getAllActionsInOrder()) {
        RockSampleAction const &rsAction =
                static_cast<RockSampleAction const &>(*action);
        GridPosition nextPosition;
        bool isLegal;
        std::tie(nextPosition, isLegal) = model_->makeNextPosition(position_, rsAction);
        if (isLegal || !model_->usingOnlyLegal()) {
            legalActions.push_back(rsAction.getBinNumber());
        }
    }
    return legalActions;
}

void PositionData::print(std::ostream &os) const {
    os << "Position: " << position_ << std::endl;
}

/* ------------------------ LegalActionsModel ----------------------- */
std::unique_ptr<solver::ActionPool> LegalActionsModel::createActionPool(
        solver::Solver */*solver*/) {
    return std::make_unique<LegalActionsPool>(this, getNumberOfBins());
}

std::unique_ptr<solver::HistoricalData> LegalActionsModel::createRootInfo() {
    RockSampleModel *model = dynamic_cast<RockSampleModel *>(this);
    return std::make_unique<PositionData>(
            model, model->getStartPosition());
}

/* ------------------------ LegalActionsPool ----------------------- */
LegalActionsPool::LegalActionsPool(solver::ModelWithDiscretizedActions *model, long numberOfBins) :
                model_(model),
                numberOfBins_(numberOfBins) {
}
std::unique_ptr<solver::ActionMapping> LegalActionsPool::createActionMapping() {
    return std::make_unique<LegalActionsMap>(model_, numberOfBins_);
}
std::unique_ptr<solver::Action> LegalActionsPool::getDefaultRolloutAction(solver::HistoricalData *data) const {
    PositionData const &positionData = static_cast<PositionData const &>(*data);
    std::vector<long> legalActions = positionData.generateLegalActions();
    long index = std::uniform_int_distribution<long>(0, legalActions.size() - 1)(
            (*model_->getRandomGenerator()));
    return model_->sampleAnAction(legalActions[index]);
}

/* ------------------------- LegalActionsMap ------------------------ */
LegalActionsMap::LegalActionsMap(solver::ModelWithDiscretizedActions *model,
        long numberOfBins) :
                    solver::DiscretizedActionMap(model, numberOfBins) {
}
void LegalActionsMap::initialize() {
    RockSampleModel &model = dynamic_cast<RockSampleModel &>(*model_);
    if (!model.usingOnlyLegal()) {
        DiscretizedActionMap::initialize();
        return;
    }

    PositionData const &data = static_cast<PositionData const &>(
                    *owningBeliefNode_->getHistoricalData());
    for (long binNumber : data.generateLegalActions()) {
        addUnvisitedAction(binNumber);
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

    RockSampleModel *model = dynamic_cast<RockSampleModel *>(getModel());
    return std::make_unique<PositionData>(model, position);
}
} /* namespace rocksample */
