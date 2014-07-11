/** @file PreferredActionsPool.hpp
 *
 * Contains an ActionPool implementation for preferred actions in RockSample.
 *
 * This is done by using the "smart" historical data stored within a PositionAndRockData instance
 * in order to determine which actions are preferred. This implementation follows the "smart"
 * data for RockSample used in POMCP, and allows for the same functionality.
 *
 * NOTE: Preferred actions are not currently updated in response to model changes!
 */
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

/** An implementation of ActionPool for preferred actions in RockSample.
 *
 * The basic functionality comes from EnumeratedActionPool; this class also customizes the
 * createBinSequence() method to control which actions are considered legal and what order they
 * are chosen in.
 * The createActionMapping() method is also modified - if preferred initialization is used, the
 * Q-values and visit counts for preferred actions can be initialized to non-zero values,
 * a la the "SMART rollout knowledge" in POMCP.
 */
class PreferredActionsPool: public solver::EnumeratedActionPool {
  public:
    /** Initializes a new PreferredActionsPool instance associated with the given model. */
    PreferredActionsPool(RockSampleModel *model);
    virtual ~PreferredActionsPool() = default;
    _NO_COPY_OR_MOVE(PreferredActionsPool);

    virtual std::vector<long> createBinSequence(solver::BeliefNode *node) override;

    virtual std::unique_ptr<solver::ActionMapping> createActionMapping(solver::BeliefNode *node)
            override;

  private:
    /** The RockSampleModel instance this pool is assoicated with. */
    RockSampleModel *model_;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_PREFERREDACTIONSPOOL_HPP_ */
