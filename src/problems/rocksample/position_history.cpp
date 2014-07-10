#include "position_history.hpp"

#include <iostream>
#include <sstream>

#include "RockSampleAction.hpp"
#include "RockSampleModel.hpp"

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"

#include "solver/abstract-problem/Action.hpp"

namespace rocksample {
/* ---------------------- PositionData --------------------- */
PositionData::PositionData(RockSampleModel *model,
        GridPosition position) :
        model_(model),
        position_(position) {
}

std::unique_ptr<solver::HistoricalData> PositionData::copy() const {
    return std::make_unique<PositionData>(model_, position_);
}

std::unique_ptr<solver::HistoricalData> PositionData::createChild(
        solver::Action const &action,
        solver::Observation const &/*observation*/) const {
    RockSampleAction const &rsAction = static_cast<RockSampleAction const &>(action);

    bool isLegal;
    GridPosition nextPosition;
    std::tie(nextPosition, isLegal) = model_->makeNextPosition(position_, rsAction.getActionType());
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
        std::tie(nextPosition, isLegal) = model_->makeNextPosition(position_,
                rsAction.getActionType());
        if (isLegal) {
            legalActions.push_back(rsAction.getBinNumber());
        }
    }
    return legalActions;
}

GridPosition PositionData::getPosition() const {
    return position_;
}

void PositionData::print(std::ostream &os) const {
    os << "Position: " << position_ << std::endl;
}

/* --------------------- PositionDataTextSerializer -------------------- */
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
