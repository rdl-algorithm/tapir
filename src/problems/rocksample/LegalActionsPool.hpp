#ifndef ROCKSAMPLE_LEGALACTIONSPOOL_HPP_
#define ROCKSAMPLE_LEGALACTIONSPOOL_HPP_

#include <memory>
#include <vector>

#include "solver/mappings/actions/enumerated_actions.hpp"
#include "problems/shared/GridPosition.hpp"

namespace solver {
class HistoricalData;
}

namespace rocksample {
class RockSampleAction;
class RockSampleModel;

class LegalActionsPool: public solver::EnumeratedActionPool {
  public:
    LegalActionsPool(RockSampleModel *model);
    virtual ~LegalActionsPool() = default;
    _NO_COPY_OR_MOVE(LegalActionsPool);

    virtual std::vector<long> createBinSequence(solver::HistoricalData const *data) override;

    /* Custom implementation of createActionMapping to keep track of our mappings. */
    virtual std::unique_ptr<solver::ActionMapping> createActionMapping(
            solver::BeliefNode *node) override;

    /* Sets the legality of the given action from the current position. */
    virtual void setLegal(bool isLegal, GridPosition position, RockSampleAction const &action);
  private:
    RockSampleModel *model_;
    std::unordered_map<GridPosition, std::unordered_set<solver::DiscretizedActionMap *>> mappings_;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_LEGALACTIONSPOOL_HPP_ */
