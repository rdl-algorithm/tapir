#ifndef ROCKSAMPLE_LEGALMAPPING_HPP_
#define ROCKSAMPLE_LEGALMAPPING_HPP_

#include "solver/mappings/discretized_actions.hpp"
#include "solver/mappings/enumerated_actions.hpp"

#include "problems/shared/GridPosition.hpp"

namespace rocksample {
class RockSampleModel;

class LegalActionsModel : virtual public solver::ModelWithEnumeratedActions {
public:
    LegalActionsModel() = default;
    virtual ~LegalActionsModel() = default;
    _NO_COPY_OR_MOVE(LegalActionsModel);

    virtual std::unique_ptr<solver::ActionPool> createActionPool(
            solver::Solver *solver) override;
};

class LegalActionsPool: public solver::DiscretizedActionPool {
  public:
    LegalActionsPool( solver::Solver *solver,
            solver::ModelWithDiscretizedActions *model, long numberOfBins);
    virtual ~LegalActionsPool() = default;
    _NO_COPY_OR_MOVE(LegalActionsPool);

    /** Creates a legal-only action mapping. */
    virtual std::unique_ptr<solver::ActionMapping> createActionMapping() override;
};

class LegalActionsMap : public solver::DiscretizedActionMap {
    friend class LegalActionsTextSerializer;
public:
    LegalActionsMap(solver::ObservationPool *observationPool,
            solver::ModelWithDiscretizedActions *model, long numberOfBins);
    virtual ~LegalActionsMap() = default;
    _NO_COPY_OR_MOVE(LegalActionsMap);

    void initialize() override;
private:
   GridPosition position_;
};

class LegalActionsTextSerializer : public solver::DiscretizedActionTextSerializer {
public:
    LegalActionsTextSerializer() = default;
    virtual ~LegalActionsTextSerializer() = default;
    virtual void saveCustomMappingData(solver::DiscretizedActionMap const &map,
            std::ostream &os) override;
    virtual void loadCustomMappingData(solver::DiscretizedActionMap &map,
                std::istream &is) override;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_LEGALMAPPING_HPP_ */
