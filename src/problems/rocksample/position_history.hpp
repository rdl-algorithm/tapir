/** @file position_history.hpp
 *
 * Defines a class to keep track of the position of the robot in RockSample.
 *
 * This is useful, since the position is fully observable but is not included in observations.
 */
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

/** A class to store the robot position associated with a given belief node.
 *
 * Since the robot position in RockSample is fully observable, all particles in any belief will
 * in fact have the same position, which is stored here.
 */
class PositionData : public solver::HistoricalData {
    friend class PositionDataTextSerializer;
public:
    /** Creates a new PositionData instance for the given model, and located in the given grid
     * square.
     */
    PositionData(RockSampleModel *model, GridPosition position);
    virtual ~PositionData() = default;
    _NO_COPY_OR_MOVE(PositionData);

    std::unique_ptr<solver::HistoricalData> copy() const;

    std::unique_ptr<solver::HistoricalData> createChild(
            solver::Action const &action,
            solver::Observation const &observation) const override;

    /** Generates the legal actions that are available from this position. */
    std::vector<long> generateLegalActions() const;

    /** Returns the grid position for this PositionData instance. */
    GridPosition getPosition() const;

    void print(std::ostream &os) const override;

private:
    /** The RockSampleModel instance this PositionData instance is associated with. */
    RockSampleModel *model_;
    /** The grid position of this PositionData. */
    GridPosition position_;
};

/** An implementation of the serialization methods for the PositionData class. */
class PositionDataTextSerializer : virtual public solver::TextSerializer {
public:
    void saveHistoricalData(solver::HistoricalData const *data, std::ostream &os) override;
    std::unique_ptr<solver::HistoricalData> loadHistoricalData(std::istream &is) override;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_POSITION_HISTORY_HPP_ */
