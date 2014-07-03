#include "LegalActionsPool.hpp"

#include <iostream>
#include <memory>

#include "solver/Solver.hpp"

#include "RockSampleModel.hpp"
#include "position_history.hpp"

namespace rocksample {
LegalActionsPool::LegalActionsPool(RockSampleModel *model) :
        EnumeratedActionPool(model, model->getAllActionsInOrder()),
        model_(model),
        mappings_() {
}

std::vector<long> LegalActionsPool::createBinSequence(solver::BeliefNode *node) {
    solver::HistoricalData const *data = node->getHistoricalData();
    RockSampleModel::RSActionCategory category = model_->getSearchActionCategory();
    if (category == RockSampleModel::RSActionCategory::LEGAL) {
        std::vector<long> bins = static_cast<PositionData const *>(data)->generateLegalActions();
        std::shuffle(bins.begin(), bins.end(), *model_->getRandomGenerator());
        return std::move(bins);
    } else {
        return EnumeratedActionPool::createBinSequence(node);
    }
}

std::unique_ptr<solver::ActionMapping> LegalActionsPool::createActionMapping(
        solver::BeliefNode *node) {
    std::unique_ptr<solver::ActionMapping> mapping = (
          DiscretizedActionPool::createActionMapping(node));

    PositionData const &data = static_cast<PositionData const &>(*node->getHistoricalData());

    solver::DiscretizedActionMap *discMap = (
            static_cast<solver::DiscretizedActionMap *>(mapping.get()));
    addMapping(data.getPosition(), discMap);

    return std::move(mapping);
}

void LegalActionsPool::addMapping(GridPosition position, solver::DiscretizedActionMap *map) {
    mappings_[position].insert(map);
}

void LegalActionsPool::setLegal(bool isLegal, GridPosition position,
        RockSampleAction const &action, solver::Solver *solver) {
    for (solver::DiscretizedActionMap *discMap : mappings_[position]) {
        // Only change affected belief nodes.
        if (solver == nullptr || solver->isAffected(discMap->getOwner())) {
            discMap->getEntry(action)->setLegal(isLegal);
        }
    }
}

/* ------------------- LegalActionsPoolTextSerializer ------------------- */
std::unique_ptr<solver::ActionMapping>
LegalActionsPoolTextSerializer::loadActionMapping(solver::BeliefNode *owner, std::istream &is) {
    std::unique_ptr<solver::ActionMapping> mapping = (
            DiscretizedActionTextSerializer::loadActionMapping(owner, is));
    solver::DiscretizedActionMap *map = static_cast<solver::DiscretizedActionMap *>(mapping.get());
    LegalActionsPool &pool = static_cast<LegalActionsPool &>(*getSolver()->getActionPool());
    GridPosition position = static_cast<PositionData &>(*owner->getHistoricalData()).getPosition();
    pool.addMapping(position, map);

    return std::move(mapping);
}
} /* namespace rocksample */
