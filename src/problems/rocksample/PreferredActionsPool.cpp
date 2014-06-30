#include "PreferredActionsPool.hpp"

#include "RockSampleAction.hpp"
#include "RockSampleModel.hpp"
#include "smart_history.hpp"

namespace rocksample {
PreferredActionsPool::PreferredActionsPool(RockSampleModel *model) :
        EnumeratedActionPool(model, model->getAllActionsInOrder()),
        model_(model) {
}

std::vector<long> PreferredActionsPool::createBinSequence(solver::BeliefNode *node) {
    solver::HistoricalData const *data = node->getHistoricalData();
    RockSampleModel::RSActionCategory category = model_->getSearchActionCategory();
    if (category == RockSampleModel::RSActionCategory::LEGAL) {
        std::vector<long> bins = static_cast<PositionAndRockData const *>(data)->generateLegalActions();
        std::shuffle(bins.begin(), bins.end(), *model_->getRandomGenerator());
        return std::move(bins);
    } else if (category == RockSampleModel::RSActionCategory::PREFERRED) {
        std::vector<long> bins = static_cast<PositionAndRockData const *>(data)->generatePreferredActions();
        std::shuffle(bins.begin(), bins.end(), *model_->getRandomGenerator());
        return std::move(bins);
    } else {
        return EnumeratedActionPool::createBinSequence(node);
    }
}

std::unique_ptr<solver::ActionMapping> PreferredActionsPool::createActionMapping(
        solver::BeliefNode *node) {
    std::unique_ptr<solver::DiscretizedActionMap> discMap = (
            std::make_unique<solver::DiscretizedActionMap>(node, this, createBinSequence(node)));

    PositionAndRockData const &data =
            static_cast<PositionAndRockData const &>(*node->getHistoricalData());

    if (model_->usingPreferredInit()) {
        for (RockSampleAction const &action : data.generatePreferredActions()) {
            long visitCount = model_->getPreferredVisitCount();
            discMap->getEntry(action)->update(visitCount, visitCount * model_->getPreferredQValue());
        }
    }

    return std::move(discMap);
}
} /* namespace rocksample */
