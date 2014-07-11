/** @file LegalActionsPool.cpp
 *
 * Contains the implementation of the mapping classes for dealing with legal actions in the
 * RockSample problem.
 */
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
    return std::make_unique<LegalActionsMap>(node, this, createBinSequence(node));
}

void LegalActionsPool::addMapping(GridPosition position, LegalActionsMap *map) {
    mappings_[position].insert(map);
}

void LegalActionsPool::removeMapping(GridPosition position, LegalActionsMap *map) {
    mappings_[position].erase(mappings_[position].find(map));
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

/* ------------------- LegalActionsMap ------------------- */
LegalActionsMap::LegalActionsMap(solver::BeliefNode *owner, LegalActionsPool *pool,
        std::vector<long> binSequence) :
        solver::DiscretizedActionMap(owner, pool, binSequence) {
    PositionData const &data = static_cast<PositionData const &>(*owner->getHistoricalData());
    pool->addMapping(data.getPosition(), this);
}

LegalActionsMap::~LegalActionsMap() {
    PositionData const &data = static_cast<PositionData const &>(*getOwner()->getHistoricalData());
    static_cast<LegalActionsPool &>(*pool_).removeMapping(data.getPosition(), this);
}

/* ------------------- LegalActionsPoolTextSerializer ------------------- */
std::unique_ptr<solver::ActionMapping>
LegalActionsPoolTextSerializer::loadActionMapping(solver::BeliefNode *owner, std::istream &is) {
    std::unique_ptr<LegalActionsMap> map = std::make_unique<LegalActionsMap>(owner,
            static_cast<LegalActionsPool *>(getSolver()->getActionPool()),
            std::vector<long> { });
    DiscretizedActionTextSerializer::loadActionMapping(*map, is);
    return std::move(map);
}
} /* namespace rocksample */
