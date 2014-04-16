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
        nextData->allRockData_[rockNo].checkCount = 10;
        nextData->allRockData_[rockNo].goodnessCount = -10;
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

std::vector<long> PositionAndRockData::generatePreferredActions() const {
    std::vector<long> preferredActions;

    int nRocks = model_->getNumberOfRocks();

    // Check if we're currently on top of a rock.
    int rockNo = model_->getCellType(position_) - RockSampleModel::ROCK;
    // If we are on top of a rock, and it has more +ve than -ve observations
    // then we will sample it.
    if (rockNo >= 0 && rockNo < nRocks) {
        RockData const &rockData = allRockData_[rockNo];
        if (rockData.chanceGood == 1.0 || rockData.goodnessCount > 0) {
            preferredActions.push_back(static_cast<long>(ActionType::SAMPLE));
            return preferredActions;
        }
    }

    bool worthwhileRockFound = false;
    bool northWorthwhile = false;
    bool southWorthwhile = false;
    bool eastWorthwhile = false;
    bool westWorthwhile = false;

    // Check to see which rocks are worthwhile.
    for (int i = 0; i < nRocks; i++) {
        RockData const &rockData = allRockData_[i];
        if (rockData.chanceGood != 0.0 && rockData.goodnessCount >= 0) {
            worthwhileRockFound = true;
            GridPosition pos = model_->getRockPosition(i);
            if (pos.i > position_.i) {
                southWorthwhile = true;
            } else if (pos.i < position_.i) {
                northWorthwhile = true;
            }

            if (pos.j > position_.j) {
                eastWorthwhile = true;
            } else if (pos.j < position_.j) {
                westWorthwhile = true;
            }
        }
    }
    // If no rocks are worthwhile head east.
    if (!worthwhileRockFound) {
        preferredActions.push_back(static_cast<long>(ActionType::EAST));
        return preferredActions;
    }

    if (northWorthwhile) {
        preferredActions.push_back(static_cast<long>(ActionType::NORTH));
    }
    if (southWorthwhile) {
        preferredActions.push_back(static_cast<long>(ActionType::SOUTH));
    }
    if (eastWorthwhile) {
        preferredActions.push_back(static_cast<long>(ActionType::EAST));
    }
    if (westWorthwhile) {
        preferredActions.push_back(static_cast<long>(ActionType::WEST));
    }

    // See which rocks we might want to check
    for (int i = 0; i < nRocks; i++) {
        RockData const &rockData = allRockData_[i];
        if (rockData.chanceGood != 0.0 && rockData.chanceGood != 1.0 &&
                std::abs(rockData.goodnessCount) < 2) {
            preferredActions.push_back(static_cast<long>(ActionType::CHECK) + i);
        }
    }
    return preferredActions;
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
        solver::Solver */*solver*/) {
    return std::make_unique<PreferredActionsPool>(this, getNumberOfBins());
}

std::unique_ptr<solver::HistoricalData> PreferredActionsModel::createRootInfo() {
    RockSampleModel *model = dynamic_cast<RockSampleModel *>(this);
    return std::make_unique<PositionAndRockData>(
            model, model->getStartPosition());
}

/* ------------------------ PreferredActionsPool ----------------------- */
PreferredActionsPool::PreferredActionsPool(solver::ModelWithDiscretizedActions *model, long numberOfBins) :
                model_(model),
                numberOfBins_(numberOfBins) {
}
std::unique_ptr<solver::ActionMapping> PreferredActionsPool::createActionMapping() {
    return std::make_unique<PreferredActionsMap>(model_, numberOfBins_);
}
std::unique_ptr<solver::Action> PreferredActionsPool::getDefaultRolloutAction(solver::HistoricalData *data) const {
    PositionAndRockData const &prData = static_cast<PositionAndRockData const &>(*data);
    std::vector<long> preferredActions = prData.generatePreferredActions();
    long index = std::uniform_int_distribution<long>(0, preferredActions.size() - 1)(
            (*model_->getRandomGenerator()));
    return model_->sampleAnAction(preferredActions[index]);
}

/* ------------------------- PreferredActionsMap ------------------------ */
PreferredActionsMap::PreferredActionsMap(
        solver::ModelWithDiscretizedActions *model, long numberOfBins) :
                    solver::DiscretizedActionMap(model, numberOfBins),
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
        preferredActions_ = data.generatePreferredActions();
        for (RockSampleAction const &action : preferredActions_) {
            long visitCount = model.getPreferredVisitCount();
            update(action, visitCount, visitCount * model.getPreferredQValue());
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

    RockSampleModel *model = dynamic_cast<RockSampleModel *>(getModel());
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
