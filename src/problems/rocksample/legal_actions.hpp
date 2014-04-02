#ifndef ROCKSAMPLE_LEGALMAPPING_HPP_
#define ROCKSAMPLE_LEGALMAPPING_HPP_

#include "solver/mappings/discretized_actions.hpp"
#include "solver/mappings/enumerated_actions.hpp"

#include "solver/search/BeliefData.hpp"

#include "solver/serialization/TextSerializer.hpp"

#include "problems/shared/GridPosition.hpp"

namespace rocksample {
class RockSampleModel;

class LegalActionsBeliefData : public solver::BeliefData {
    friend class LegalActionsMap;
    friend class LegalActionsTextSerializer;
public:
    LegalActionsBeliefData(RockSampleModel *model,
            GridPosition position);
    virtual ~LegalActionsBeliefData() = default;
    _NO_COPY_OR_MOVE(LegalActionsBeliefData);

    std::unique_ptr<solver::BeliefData> createChildData(
            solver::Action const &action,
            solver::Observation const &observation);
private:
    RockSampleModel *model_;
    GridPosition position_;
};

class LegalActionsModel : virtual public solver::ModelWithEnumeratedActions {
public:
    LegalActionsModel() = default;
    virtual ~LegalActionsModel() = default;
    _NO_COPY_OR_MOVE(LegalActionsModel);

    virtual std::unique_ptr<solver::ActionPool> createActionPool(
            solver::Solver *solver) override;
    virtual std::unique_ptr<solver::BeliefData> createRootBeliefData() override;
};

class LegalActionsPool: public solver::DiscretizedActionPool {
  public:
    LegalActionsPool(solver::Solver *solver,
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
            solver::ModelWithDiscretizedActions *model,
            long numberOfBins);
    virtual ~LegalActionsMap() = default;
    _NO_COPY_OR_MOVE(LegalActionsMap);

    void initialize() override;
};

class LegalActionsTextSerializer : virtual public solver::TextSerializer,
virtual public solver::DiscretizedActionTextSerializer {
public:
    void saveBeliefData(solver::BeliefData const *data, std::ostream &os) override;
    std::unique_ptr<solver::BeliefData> loadBeliefData(std::istream &is) override;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_LEGALMAPPING_HPP_ */
