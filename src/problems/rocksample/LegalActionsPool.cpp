#include "LegalActionsPool.hpp"

#include <memory>

#include "RockSampleModel.hpp"
#include "position_history.hpp"

namespace rocksample {
LegalActionsPool::LegalActionsPool(RockSampleModel *model) :
        EnumeratedActionPool(model, model->getAllActionsInOrder()),
        model_(model),
        mappings_() {
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

std::unique_ptr<solver::ActionMapping> LegalActionsPool::createActionMapping(
        solver::BeliefNode *node) {
    std::unique_ptr<solver::ActionMapping> mapping = (
            DiscretizedActionPool::createActionMapping(node));

    PositionData const &data = static_cast<PositionData const &>(*node->getHistoricalData());

    solver::DiscretizedActionMap *discMap = (
            static_cast<solver::DiscretizedActionMap *>(mapping.get()));
    mappings_[data.getPosition()].insert(discMap);

    return std::move(mapping);
}

void LegalActionsPool::setLegal(bool isLegal, GridPosition position,
        RockSampleAction const &action) {
    for (solver::DiscretizedActionMap *discMap : mappings_[position]) {
        discMap->getEntry(action)->setLegal(isLegal);
    }
}
} /* namespace rocksample */
