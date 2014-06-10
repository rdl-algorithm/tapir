#include "solver/mappings/actions/enumerated_actions.hpp"

namespace solver {
/* ------------------- EnumeratedActionPool ------------------- */
EnumeratedActionPool::EnumeratedActionPool(Model *model,
        std::vector<std::unique_ptr<DiscretizedPoint>> allActions) :
        DiscretizedActionPool(model),
        model_(model),
        allActions_(std::move(allActions)) {
}
long EnumeratedActionPool::getNumberOfBins() {
    return allActions_.size();
}
std::unique_ptr<Action> EnumeratedActionPool::sampleAnAction(
        long binNumber) {
    return allActions_[binNumber]->copy();
}

std::vector<long> EnumeratedActionPool::createBinSequence(HistoricalData const */*data*/) {
    std::vector<long> bins;
    for (int i = 0; i < getNumberOfBins(); i++) {
        bins.push_back(i);
    }
    std::shuffle(bins.begin(), bins.end(), *model_->getRandomGenerator());
    return std::move(bins);
}
} /* namespace solver */



