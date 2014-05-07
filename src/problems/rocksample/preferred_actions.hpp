#ifndef ROCKSAMPLE_PREFERREDACTIONS_HPP_
#define ROCKSAMPLE_PREFERREDACTIONS_HPP_

#include "solver/mappings/discretized_actions.hpp"
#include "solver/mappings/enumerated_actions.hpp"

#include "solver/search/HistoricalData.hpp"

#include "solver/serialization/TextSerializer.hpp"

#include "problems/shared/GridPosition.hpp"

namespace rocksample {
class RockSampleAction;
class RockSampleModel;

struct RockData {
    long checkCount = 0;
    long goodnessCount = 0;

    double chanceGood = 0.5;
};

class PositionAndRockData : public solver::HistoricalData {
    friend class PreferredActionsMap;
    friend class PreferredActionsTextSerializer;
public:
    PositionAndRockData(RockSampleModel *model, GridPosition position);
    virtual ~PositionAndRockData() = default;

    // We allow copying via copy constructor only.
    PositionAndRockData(PositionAndRockData const &other);
    PositionAndRockData(PositionAndRockData &&other) = delete;
    PositionAndRockData &operator=(PositionAndRockData const &other) = delete;
    PositionAndRockData &operator=(PositionAndRockData &&other) = delete;

    std::unique_ptr<solver::HistoricalData> createChild(
            solver::Action const &action,
            solver::Observation const &observation) override;

    std::vector<long> generatePreferredActions() const;

    void print(std::ostream &os) const override;

private:
    RockSampleModel *model_;
    GridPosition position_;
    std::vector<RockData> allRockData_;
};

class PreferredActionsModel : virtual public solver::ModelWithEnumeratedActions {
public:
    PreferredActionsModel() = default;
    virtual ~PreferredActionsModel() = default;
    _NO_COPY_OR_MOVE(PreferredActionsModel);

    virtual std::unique_ptr<solver::ActionPool> createActionPool(
            solver::Solver *solver) override;
    virtual std::unique_ptr<solver::HistoricalData> createRootHistoricalData() override;
};

class PreferredActionsPool: public solver::ActionPool {
  public:
    PreferredActionsPool(solver::ModelWithDiscretizedActions *model, long numberOfBins);
    virtual ~PreferredActionsPool() = default;
    _NO_COPY_OR_MOVE(PreferredActionsPool);

    /** Creates a preferred-only action mapping. */
    virtual std::unique_ptr<solver::ActionMapping> createActionMapping() override;

    /** Selects a random preferred action for the rollout. */
    virtual std::unique_ptr<solver::Action> getDefaultRolloutAction(solver::HistoricalData *data) const override;
  private:
      solver::ModelWithDiscretizedActions *model_;
      long numberOfBins_;
};

class PreferredActionsMap : public solver::DiscretizedActionMap {
    friend class PreferredActionsTextSerializer;
public:
    PreferredActionsMap(solver::ModelWithDiscretizedActions *model,
            long numberOfBins);
    virtual ~PreferredActionsMap() = default;
    _NO_COPY_OR_MOVE(PreferredActionsMap);

    void initialize() override;
};

class PreferredActionsTextSerializer : virtual public solver::TextSerializer,
virtual public solver::DiscretizedActionTextSerializer {
public:
    void saveHistoricalData(solver::HistoricalData const *data, std::ostream &os) override;
    std::unique_ptr<solver::HistoricalData> loadHistoricalData(std::istream &is) override;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_PREFERREDACTIONS_HPP_ */
