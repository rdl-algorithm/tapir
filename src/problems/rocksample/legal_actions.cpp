#include "legal_actions.hpp"

#include <iostream>
#include <sstream>

#include "RockSampleAction.hpp"
#include "RockSampleModel.hpp"

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"

#include "solver/abstract-problem/Action.hpp"

#include "solver/mappings/actions/ActionMapping.hpp"
#include "solver/mappings/observations/ObservationMapping.hpp"

namespace rocksample {
/* ---------------------- PositionData --------------------- */
PositionData::PositionData(RockSampleModel *model,
        GridPosition position) :
        model_(model),
        position_(position) {
}

std::unique_ptr<solver::HistoricalData> PositionData::createChild(
        solver::Action const &action,
        solver::Observation const &/*observation*/) const {
    RockSampleAction const &rsAction = static_cast<RockSampleAction const &>(action);

    bool isLegal;
    GridPosition nextPosition;
    std::tie(nextPosition, isLegal) = model_->makeNextPosition(position_, rsAction);
    if (!isLegal) {
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
        if (isLegal) {
            legalActions.push_back(rsAction.getBinNumber());
        }
    }
    return legalActions;
}

void PositionData::print(std::ostream &os) const {
    os << "Position: " << position_ << std::endl;
}

/* ------------------------ LegalActionsPool ----------------------- */
LegalActionsPool::LegalActionsPool(RockSampleModel *model) :
        EnumeratedActionPool(model, model->getAllActionsInOrder()),
        model_(model) {
}

std::vector<long> LegalActionsPool::createBinSequence(solver::HistoricalData const *data) {
    RockSampleModel::RSActionCategory category = model_->getSearchActionCategory();
    if (category == RockSampleModel::RSActionCategory::LEGAL) {
        std::vector<long> bins = static_cast<PositionData const *>(data)->generateLegalActions();
        std::shuffle(bins.begin(), bins.end(), *model_->getRandomGenerator());
        return std::move(bins);
    } else {
        return EnumeratedActionPool::createBinSequence(data);
    }
}

/* --------------------- LegalActionsTextSerializer -------------------- */
void PositionDataTextSerializer::saveHistoricalData(solver::HistoricalData const *data,
        std::ostream &os) {
    os << std::endl;
    os << "CUSTOM DATA:" << std::endl;
    PositionData const &legalData = (
                static_cast<PositionData const &>(*data));
    os << "Position: " << legalData.position_ << std::endl;
    os << std::endl;
}
std::unique_ptr<solver::HistoricalData> PositionDataTextSerializer::loadHistoricalData(
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
