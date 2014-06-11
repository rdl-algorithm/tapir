#ifndef ROCKSAMPLE_LEGALACTIONSPOOL_HPP_
#define ROCKSAMPLE_LEGALACTIONSPOOL_HPP_

#include <memory>
#include <vector>

#include "solver/mappings/actions/enumerated_actions.hpp"

namespace solver {
class HistoricalData;
}

namespace rocksample {
class RockSampleModel;

class LegalActionsPool: public solver::EnumeratedActionPool {
  public:
    LegalActionsPool(RockSampleModel *model);
    virtual ~LegalActionsPool() = default;
    _NO_COPY_OR_MOVE(LegalActionsPool);

    virtual std::vector<long> createBinSequence(solver::HistoricalData const *data) override;
  private:
    RockSampleModel *model_;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_LEGALACTIONSPOOL_HPP_ */
