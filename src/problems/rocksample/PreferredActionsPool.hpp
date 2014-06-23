#ifndef ROCKSAMPLE_PREFERREDACTIONSPOOL_HPP_
#define ROCKSAMPLE_PREFERREDACTIONSPOOL_HPP_

#include <memory>
#include <vector>

#include "solver/mappings/actions/enumerated_actions.hpp"

namespace solver {
class ActionMapping;
class BeliefNode;
class HistoricalData;
}

namespace rocksample {
class RockSampleModel;

class PreferredActionsPool: public solver::EnumeratedActionPool {
  public:
    PreferredActionsPool(RockSampleModel *model);
    virtual ~PreferredActionsPool() = default;
    _NO_COPY_OR_MOVE(PreferredActionsPool);

    virtual std::vector<long> createBinSequence(solver::HistoricalData const *data) override;

    virtual std::unique_ptr<solver::ActionMapping> createActionMapping(solver::BeliefNode *node)
            override;
  private:
    RockSampleModel *model_;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_PREFERREDACTIONSPOOL_HPP_ */
