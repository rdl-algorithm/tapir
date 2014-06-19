#ifndef ROCKSAMPLE_LEGALACTIONSPOOL_HPP_
#define ROCKSAMPLE_LEGALACTIONSPOOL_HPP_

#include <memory>
#include <vector>

#include "solver/mappings/actions/enumerated_actions.hpp"
#include "problems/shared/GridPosition.hpp"

namespace solver {
class HistoricalData;
class Solver;
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

    /** Adds a mapping to the saved entries for a given grid position. */
    virtual void addMapping(GridPosition position, solver::DiscretizedActionMap *map);

    /* Sets the legality of the given action from the current position.
     *
     * If the solver is not nullptr, it will be queried for which belief nodes are affected
     * by the current changes, so that only affected belief nodes will be modified. */
    virtual void setLegal(bool isLegal, GridPosition position, RockSampleAction const &action,
            solver::Solver *solver);
  private:
    RockSampleModel *model_;
    std::unordered_map<GridPosition, std::unordered_set<solver::DiscretizedActionMap *>> mappings_;
};

class LegalActionsPoolTextSerializer: virtual public solver::DiscretizedActionTextSerializer {
  public:
    LegalActionsPoolTextSerializer() = default;
    virtual ~LegalActionsPoolTextSerializer() = default;
    _NO_COPY_OR_MOVE(LegalActionsPoolTextSerializer);

    virtual std::unique_ptr<solver::ActionMapping> loadActionMapping(solver::BeliefNode *node,
            std::istream &is) override;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_LEGALACTIONSPOOL_HPP_ */
