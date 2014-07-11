/** @file RockSampleAction.hpp
 *
 * Defines the RockSampleAction class, which represents an action for the RockSample problem, and
 * also the ActionType enumeration, which enumerates the different types of actions for RockSample.
 */
#ifndef ROCKSAMPLE_ACTION_HPP_
#define ROCKSAMPLE_ACTION_HPP_

#include <cstddef>                      // for size_t
#include <cstdint>

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/DiscretizedPoint.hpp"             // for DiscretizedPoint

namespace rocksample {
/** An enumeration of the possible action types in RockSample.
 */
enum class ActionType : uint8_t {
    /** Move north. */
    NORTH = 0,
    /** Move east. */
    EAST = 1,
    /** Move south. */
    SOUTH = 2,
    /** Move west. */
    WEST = 3,
    /** Sample a rock. */
    SAMPLE = 4,
    /** Check one of the rocks on the map using the sensor. */
    CHECK = 5
};

/** An insertion operator, for human-readable printing of action types. */
inline std::ostream &operator<<(std::ostream &os, ActionType const actionType) {
    switch (actionType) {
    case ActionType::CHECK:
        os << "CHECK-";
        break;
    case ActionType::NORTH:
        os << "NORTH";
        break;
    case ActionType::EAST:
        os << "EAST";
        break;
    case ActionType::SOUTH:
        os << "SOUTH";
        break;
    case ActionType::WEST:
        os << "WEST";
        break;
    case ActionType::SAMPLE:
        os << "SAMPLE";
        break;
    default:
        os << "ERROR-" << static_cast<long>(actionType);
        break;
    }
    return os;
}

/** A class representing an action in the RockSample POMDP
 *
 * This class also implements solver::DiscretizedPoint so that the solver can use a simplistic
 * enumerated action mapping approach (EnumeratedActionPool) to store the available actions from
 * each belief node.
 */
class RockSampleAction : public solver::DiscretizedPoint {
    friend class RockSampleTextSerializer;
  public:
    /** Constructs a new action from the given ActionType. */
    RockSampleAction(ActionType actionType, uint8_t rockNo = 0);
    /** Constructs a new action from the given integer code. */
    RockSampleAction(long code);
    virtual ~RockSampleAction() = default;

    std::unique_ptr<solver::Action> copy() const override;
    double distanceTo(solver::Action const &otherAction) const override;
    void print(std::ostream &os) const override;

    long getBinNumber() const override;

    /** Returns the ActionType of this action. */
    ActionType getActionType() const;
    /** If the ActionType is CHECK, this returns the rock number being checked; otherwise this
     * return value is arbitrary.
     */
    long getRockNo() const;

  private:
    /** The type of action this is. */
    ActionType actionType_;
    /** If the action type is CHECK, this will be the rock number of the rock to be checked. */
    uint8_t rockNo_;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_ACTION_HPP_ */
