#ifndef ROCKSAMPLE_ACTION_HPP_
#define ROCKSAMPLE_ACTION_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/DiscretizedPoint.hpp"             // for DiscretizedPoint

namespace rocksample {
enum class ActionType : long {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
    SAMPLE = 4,
    CHECK = 5
};

class RockSampleAction : public solver::DiscretizedPoint {
    friend class RockSampleTextSerializer;
  public:
    RockSampleAction(ActionType actionType, long rockNo = 0);
    RockSampleAction(long code);
    virtual ~RockSampleAction() = default;

    std::unique_ptr<solver::Action> copy() const override;
    double distanceTo(solver::Action const &otherAction) const override;
    void print(std::ostream &os) const override;

    long getBinNumber() const override;

    ActionType getActionType() const;
    long getRockNo() const;
  private:
    ActionType actionType_;
    long rockNo_;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_ACTION_HPP_ */
