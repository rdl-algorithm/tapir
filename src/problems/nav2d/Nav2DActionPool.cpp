#include "Nav2DActionPool.hpp"

#include "Nav2DAction.hpp"
#include "Nav2DModel.hpp"

namespace nav2d {
Nav2DActionPool::Nav2DActionPool(Nav2DModel *model) :
        model_(model) {
}

long Nav2DActionPool::getNumberOfBins() {
    return Nav2DAction::getNumberOfBins();
}

std::unique_ptr<solver::Action> Nav2DActionPool::sampleAnAction(long binNumber) {
    return std::make_unique<Nav2DAction>(binNumber, model_);

}

std::vector<long> Nav2DActionPool::createBinSequence(solver::HistoricalData const */*data*/) {
    std::vector<long> bins;
    for (int i = 0; i < getNumberOfBins(); i++) {
        bins.push_back(i);
    }
    std::shuffle(bins.begin(), bins.end(), *model_->getRandomGenerator());
    return std::move(bins);
}


} /* namespace nav2d */
