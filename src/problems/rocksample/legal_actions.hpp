#ifndef ROCKSAMPLE_LEGALACTIONS_HPP_
#define ROCKSAMPLE_LEGALACTIONS_HPP_

#include "solver/abstract-problem/HistoricalData.hpp"

#include "solver/mappings/actions/discretized_actions.hpp"
#include "solver/mappings/actions/enumerated_actions.hpp"

#include "solver/serialization/TextSerializer.hpp"

#include "problems/shared/GridPosition.hpp"

namespace rocksample {
class RockSampleModel;

class PositionData : public solver::HistoricalData {
    friend class LegalActionsMap;
    friend class LegalActionsTextSerializer;
public:
    PositionData(RockSampleModel *model, GridPosition position);
    virtual ~PositionData() = default;
    _NO_COPY_OR_MOVE(PositionData);

    std::unique_ptr<solver::HistoricalData> createChild(
            solver::Action const &action,
            solver::Observation const &observation) override;

    std::vector<long> generateLegalActions() const;

    void print(std::ostream &os) const override;

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
    virtual std::unique_ptr<solver::HistoricalData> createRootHistoricalData() override;
};

class LegalActionsPool: public solver::ActionPool {
  public:
    LegalActionsPool(solver::ModelWithDiscretizedActions *model, long numberOfBins);
    virtual ~LegalActionsPool() = default;
    _NO_COPY_OR_MOVE(LegalActionsPool);

    /** Creates a legal-only action mapping. */
    virtual std::unique_ptr<solver::ActionMapping> createActionMapping() override;

    /** Selects a random legal action for the rollout. */
    virtual std::unique_ptr<solver::Action> getDefaultRolloutAction(solver::HistoricalData *data) const override;

private:
    solver::ModelWithDiscretizedActions *model_;
    long numberOfBins_;
};

class LegalActionsMap : public solver::DiscretizedActionMap {
    friend class LegalActionsTextSerializer;
public:
    LegalActionsMap(solver::ModelWithDiscretizedActions *model, long numberOfBins);
    virtual ~LegalActionsMap() = default;
    _NO_COPY_OR_MOVE(LegalActionsMap);

    void initialize() override;
};

class LegalActionsTextSerializer : virtual public solver::TextSerializer,
virtual public solver::DiscretizedActionTextSerializer {
public:
    void saveHistoricalData(solver::HistoricalData const *data, std::ostream &os) override;
    std::unique_ptr<solver::HistoricalData> loadHistoricalData(std::istream &is) override;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_LEGALACTIONS_HPP_ */
