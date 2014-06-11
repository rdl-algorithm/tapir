#include "LegalActionsPool.hpp"

#include "RockSampleModel.hpp"
#include "position_history.hpp"

namespace rocksample {
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
} /* namespace rocksample */
