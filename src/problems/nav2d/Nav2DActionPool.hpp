#ifndef NAV2D_ACTIONPOOL_HPP_
#define NAV2D_ACTIONPOOL_HPP_

#include "global.hpp"

#include "solver/mappings/actions/discretized_actions.hpp"

namespace nav2d {
class Nav2DModel;
class Nav2DSpcHistoryCorrector;

class Nav2DActionPool: public solver::DiscretizedActionPool {
public:
    Nav2DActionPool(Nav2DModel *model);
    virtual ~Nav2DActionPool() = default;
    _NO_COPY_OR_MOVE(Nav2DActionPool);

    virtual long getNumberOfBins() override;
    virtual std::unique_ptr<solver::Action> sampleAnAction(long binNumber) override;
    virtual std::vector<long> createBinSequence(solver::HistoricalData const *data) override;
private:
    Nav2DModel *model_;
};
} /* namespace nav2d */

#endif /* NAV2D_ACTIONPOOL_HPP_ */
