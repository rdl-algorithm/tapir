#include "preferred_actions.hpp"

#include <iostream>
#include <sstream>

#include "RockSampleAction.hpp"
#include "RockSampleObservation.hpp"
#include "RockSampleModel.hpp"

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"

#include "solver/abstract-problem/Action.hpp"

#include "solver/mappings/ActionMapping.hpp"
#include "solver/mappings/ObservationMapping.hpp"

namespace rocksample {
/* ---------------------- PositionAndRockData --------------------- */
PositionAndRockData::PositionAndRockData(RockSampleModel *model,
        GridPosition position) :
        model_(model),
        position_(position),
        allRockData_(model_->getNumberOfRocks()) {
}

PositionAndRockData::PositionAndRockData(PositionAndRockData const &other) :
        model_(other.model_),
        position_(other.position_),
        allRockData_(other.allRockData_) {
}

std::unique_ptr<solver::HistoricalData> PositionAndRockData::createChild(
        solver::Action const &action,
        solver::Observation const &observation) {
    RockSampleAction const &rsAction = static_cast<RockSampleAction const &>(action);

    std::unique_ptr<PositionAndRockData> nextData = (
            std::make_unique<PositionAndRockData>(*this));

    bool isLegal;
    std::tie(nextData->position_, isLegal) = model_->makeNextPosition(position_, rsAction);
    if (!isLegal && model_->usingOnlyLegal()) {
        debug::show_message("ERROR: An illegal action was taken!?");
        return std::move(nextData);
    }

    if (rsAction.getActionType() == ActionType::SAMPLE) {
        int rockNo = model_->getCellType(position_) - RockSampleModel::ROCK;
        nextData->allRockData_[rockNo].chanceGood = 0.0;
    } else if (rsAction.getActionType() == ActionType::CHECK) {
        int rockNo = rsAction.getRockNo();

        GridPosition rockPos = model_->getRockPosition(rockNo);
        double distance = position_.euclideanDistanceTo(rockPos);
        double probabilityCorrect = (
                model_->getSensorCorrectnessProbability(distance));
        double probabilityIncorrect = 1 - probabilityCorrect;

        RockSampleObservation const &rsObs = (
                static_cast<RockSampleObservation const &>(observation));

        RockData &rockData = nextData->allRockData_[rockNo];
        rockData.checkCount++;
        double likelihoodGood = rockData.chanceGood;
        double likelihoodBad = 1 - rockData.chanceGood;
        if (rsObs.isGood()) {
            rockData.goodnessCount++;
            likelihoodGood *= probabilityCorrect;
            likelihoodBad *= probabilityIncorrect;
        } else {
            rockData.goodnessCount--;
            likelihoodGood *= probabilityIncorrect;
            likelihoodBad *= probabilityCorrect;
        }
        rockData.chanceGood = likelihoodGood / (likelihoodGood + likelihoodBad);
    }
    return std::move(nextData);
}

void PositionAndRockData::print(std::ostream &os) const {
    os << "Position: " << position_ << std::endl;
    os << "Chances of goodness: ";
    for (RockData const &rockData : allRockData_) {
        abt::print_double(rockData.chanceGood, os, 6, 4);
        os << " ";
    }
    os << std::endl;
}

/* ------------------------ PreferredActionsModel ----------------------- */
std::unique_ptr<solver::ActionPool> PreferredActionsModel::createActionPool(
        solver::Solver *solver) {
    return std::make_unique<PreferredActionsPool>(solver, this, getNumberOfBins());
}

std::unique_ptr<solver::HistoricalData> PreferredActionsModel::createRootInfo() {
    RockSampleModel *model = dynamic_cast<RockSampleModel *>(this);
    return std::make_unique<PositionAndRockData>(
            model, model->getStartPosition());
}

/* ------------------------ PreferredActionsPool ----------------------- */
PreferredActionsPool::PreferredActionsPool(solver::Solver *solver,
        solver::ModelWithDiscretizedActions *model, long numberOfBins) :
                solver::DiscretizedActionPool(solver, model, numberOfBins) {
}
std::unique_ptr<solver::ActionMapping> PreferredActionsPool::createActionMapping() {
    return std::make_unique<PreferredActionsMap>(
            getSolver()->getObservationPool(), model_, numberOfBins_);
}

/* ------------------------- PreferredActionsMap ------------------------ */
PreferredActionsMap::PreferredActionsMap(solver::ObservationPool *observationPool,
            solver::ModelWithDiscretizedActions *model, long numberOfBins) :
                    solver::DiscretizedActionMap(observationPool, model, numberOfBins),
                    preferredActions_() {
}

void PreferredActionsMap::initialize() {
    RockSampleModel &model = dynamic_cast<RockSampleModel &>(*model_);
    PositionAndRockData const &data = static_cast<PositionAndRockData const &>(
                    *owningBeliefNode_->getHistoricalData());

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

    if (model.usingPreferredInit()) {
        generatePreferredActions();
        for (RockSampleAction const &action : preferredActions_) {
            long visitCount = model.getPreferredVisitCount();
            update(action, visitCount, visitCount * model.getPreferredQValue());
        }
    }
}

std::vector<RockSampleAction> PreferredActionsMap::getPreferredActions() const {
    return preferredActions_;
}

std::unique_ptr<RockSampleAction> PreferredActionsMap::getRandomPreferredAction() const {
    int index = std::uniform_int_distribution<int>(
            0, preferredActions_.size() - 1)(*model_->getRandomGenerator());
    return std::make_unique<RockSampleAction>(preferredActions_[index]);
}

void PreferredActionsMap::generatePreferredActions() {
    RockSampleModel &model = dynamic_cast<RockSampleModel &>(*model_);
    PositionAndRockData const &data = static_cast<PositionAndRockData const &>(
            *owningBeliefNode_->getHistoricalData());

    preferredActions_.clear();

    int nRocks = model.getNumberOfRocks();
    int rockNo = model.getCellType(data.position_) - RockSampleModel::ROCK;

    // If the rock has more +ve than -ve observations we should sample it.
    if (rockNo >= 0 && rockNo < nRocks) {
        if (data.allRockData_[rockNo].goodnessCount > 0) {
            preferredActions_.push_back(RockSampleAction(ActionType::SAMPLE));
            return;
        }
    }

    bool worthwhileRockFound = false;
    bool northWorthwhile = false;
    bool southWorthwhile = false;
    bool eastWorthwhile = false;
    bool westWorthwhile = false;

    // Check to see which rocks are worthwhile.
    for (int i = 0; i < nRocks; i++) {
        RockData const &rockData = data.allRockData_[i];
        if (rockData.goodnessCount >= 0) {
            worthwhileRockFound = true;
            GridPosition pos = model.getRockPosition(i);
            if (pos.i > data.position_.i) {
                southWorthwhile = true;
            } else if (pos.i < data.position_.i) {
                northWorthwhile = true;
            }

            if (pos.j > data.position_.j) {
                eastWorthwhile = true;
            } else if (pos.j < data.position_.j) {
                westWorthwhile = true;
            }
        }
    }
    if (!worthwhileRockFound) {
        preferredActions_.push_back(RockSampleAction(ActionType::EAST));
        return;
    }
    if (northWorthwhile) {
        preferredActions_.push_back(RockSampleAction(ActionType::NORTH));
    }
    if (southWorthwhile) {
        preferredActions_.push_back(RockSampleAction(ActionType::SOUTH));
    }
    if (eastWorthwhile) {
        preferredActions_.push_back(RockSampleAction(ActionType::EAST));
    }
    if (westWorthwhile) {
        preferredActions_.push_back(RockSampleAction(ActionType::WEST));
    }

    // See which rocks we might want to check
    for (int i = 0; i < nRocks; i++) {
        RockData const &rockData = data.allRockData_[i];
        if (rockData.chanceGood != 0.0 && rockData.chanceGood != 1.0 &&
                rockData.checkCount < 5 && std::abs(rockData.goodnessCount) < 2) {
            preferredActions_.push_back(RockSampleAction(ActionType::CHECK, i));
        }
    }
}

/* --------------------- PreferredActionsTextSerializer -------------------- */
void PreferredActionsTextSerializer::saveHistoricalData(
        solver::HistoricalData const *data, std::ostream &os) {
    os << std::endl;
    os << "CUSTOM DATA:" << std::endl;
    PositionAndRockData const &prData = (
                static_cast<PositionAndRockData const &>(*data));
    os << "Position: " << prData.position_ << std::endl;
    for (RockData const &rockData : prData.allRockData_) {
        os << "p = ";
        abt::print_double(rockData.chanceGood, os, 7, 5);
        os << " from " << rockData.checkCount << " checks ( ";
        os << std::showpos << rockData.goodnessCount << std::noshowpos;
        os << " )" << std::endl;
    }
    os << std::endl;
}
std::unique_ptr<solver::HistoricalData> PreferredActionsTextSerializer::loadHistoricalData(
        std::istream &is) {

    std::string line;
    std::getline(is, line); // Blank line
    std::getline(is, line); // Header

    std::getline(is, line);
    std::string tmpStr;
    GridPosition position;
    std::istringstream(line) >> tmpStr >> position;

    RockSampleModel *model = dynamic_cast<RockSampleModel *>(model_);
    std::unique_ptr<PositionAndRockData> data = (
            std::make_unique<PositionAndRockData>(model, position));

    for (RockData &rockData : data->allRockData_) {
        std::getline(is, line);
        std::istringstream sstr(line);

        sstr >> tmpStr >> tmpStr >> rockData.chanceGood;
        sstr >> tmpStr >> rockData.checkCount >> tmpStr >> tmpStr;
        sstr >> rockData.goodnessCount;
    }

    std::getline(is, line); // Blank line

    return std::move(data);
}
} /* namespace rocksample */
