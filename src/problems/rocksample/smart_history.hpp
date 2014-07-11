/** @file smart_history.hpp
 *
 * Defines a class to keep track of the position of the robot in RockSample, as well as explicitly
 * calculated probabilities for the goodness of each rock.
 */
#ifndef ROCKSAMPLE_SMART_HISTORY_HPP_
#define ROCKSAMPLE_SMART_HISTORY_HPP_

#include "solver/abstract-problem/HistoricalData.hpp"

#include "solver/serialization/TextSerializer.hpp"

#include "problems/shared/GridPosition.hpp"

namespace rocksample {
class RockSampleAction;
class RockSampleModel;

/** Stores data about each rock.
 */
struct RockData {
    /** The number of times this rock has been checked. */
    long checkCount = 0;
    /** The "goodness number"; +1 for each good observation of this rock, and -1 for each bad
     * observation of this rock.
     */
    long goodnessNumber = 0;
    /** The calculated probability that this rock is good. */
    double chanceGood = 0.5;
};

/** A class to store the robot position associated with a given belief node, as well as
 * explicitly calculated probabilities of goodness for each rock.
 */
class PositionAndRockData : public solver::HistoricalData {
    friend class PreferredActionsMap;
    friend class PositionAndRockDataTextSerializer;
public:
    /** Constructs a new PositionAndRockData instance for the given model, in the given position,
     * with default data values for each rock.
     */
    PositionAndRockData(RockSampleModel *model, GridPosition position);
    virtual ~PositionAndRockData() = default;

    /** We define a copy constructor for this class, for convenience. */
    PositionAndRockData(PositionAndRockData const &other);
    /** Deleted move constructor. */
    PositionAndRockData(PositionAndRockData &&other) = delete;
    /** Deleted copy assignment operator. */
    PositionAndRockData &operator=(PositionAndRockData const &other) = delete;
    /** Deleted move assignment operator. */
    PositionAndRockData &operator=(PositionAndRockData &&other) = delete;

    std::unique_ptr<solver::HistoricalData> copy() const;

    std::unique_ptr<solver::HistoricalData> createChild(
            solver::Action const &action,
            solver::Observation const &observation) const override;

    /** Generates the legal actions that are available from this position. */
    std::vector<long> generateLegalActions() const;
    /** Generates a set of preferred actions, based on the knowledge stored in this
     * instance.
     */
    std::vector<long> generatePreferredActions() const;

    void print(std::ostream &os) const override;

private:
    /** The associated model instance. */
    RockSampleModel *model_;
    /** The grid position. */
    GridPosition position_;
    /** The data for each rock, in order of rock number. */
    std::vector<RockData> allRockData_;
};

/** An implementation of the serialization methods for the
 * PositionAndRockDataTextSerializer class.
 */
class PositionAndRockDataTextSerializer : virtual public solver::TextSerializer {
public:
    void saveHistoricalData(solver::HistoricalData const *data, std::ostream &os) override;
    std::unique_ptr<solver::HistoricalData> loadHistoricalData(std::istream &is) override;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_SMART_HISTORY_HPP_ */
