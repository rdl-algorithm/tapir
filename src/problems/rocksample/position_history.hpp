#ifndef ROCKSAMPLE_POSITION_HISTORY_HPP_
#define ROCKSAMPLE_POSITION_HISTORY_HPP_

#include <memory>
#include <vector>

#include "solver/abstract-problem/HistoricalData.hpp"

#include "solver/serialization/TextSerializer.hpp"

#include "problems/shared/GridPosition.hpp"

namespace rocksample {
class RockSampleAction;
class RockSampleModel;

class PositionData : public solver::HistoricalData {
    friend class PositionDataTextSerializer;
public:
    PositionData(RockSampleModel *model, GridPosition position);
    virtual ~PositionData() = default;
    _NO_COPY_OR_MOVE(PositionData);

    std::unique_ptr<solver::HistoricalData> createChild(
            solver::Action const &action,
            solver::Observation const &observation) const override;

    std::vector<long> generateLegalActions() const;
    GridPosition getPosition() const;

    void print(std::ostream &os) const override;
private:
    RockSampleModel *model_;
    GridPosition position_;
};

class PositionDataTextSerializer : virtual public solver::TextSerializer {
public:
    void saveHistoricalData(solver::HistoricalData const *data, std::ostream &os) override;
    std::unique_ptr<solver::HistoricalData> loadHistoricalData(std::istream &is) override;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_POSITION_HISTORY_HPP_ */
