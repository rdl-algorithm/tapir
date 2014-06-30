#ifndef ROCKSAMPLE_ACTION_HPP_
#define ROCKSAMPLE_ACTION_HPP_

#include <cstddef>                      // for size_t
#include <cstdint>

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/DiscretizedPoint.hpp"             // for DiscretizedPoint

namespace rocksample {
typedef uint8_t ActionTypeIntType;

enum class ActionType : ActionTypeIntType {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
    SAMPLE = 4,
    CHECK = 5
};

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
        os << "ERROR-" << static_cast<ActionTypeIntType>(actionType);
        break;
    }
    return os;
}

class RockSampleAction : public solver::DiscretizedPoint {
    friend class RockSampleTextSerializer;
  public:
    RockSampleAction(ActionType actionType, uint8_t rockNo = 0);
    RockSampleAction(long code);
    virtual ~RockSampleAction() = default;

    std::unique_ptr<solver::Action> copy() const override;
    double distanceTo(solver::Action const &otherAction) const override;
    void print(std::ostream &os) const override;

    long getBinNumber() const override;

    ActionType getActionType() const;
    uint8_t getRockNo() const;
  private:
    ActionType actionType_;
    uint8_t rockNo_;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_ACTION_HPP_ */
