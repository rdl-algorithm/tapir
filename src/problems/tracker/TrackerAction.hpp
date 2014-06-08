#ifndef TRACKER_ACTION_HPP_
#define TRACKER_ACTION_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/DiscretizedPoint.hpp"             // for DiscretizedPoint

namespace tracker {
enum class ActionType : long {
    FORWARD = 0,
    TURN_RIGHT = 1,
    TURN_LEFT = 2,
    WAIT = 3,
    REVERSE = 4,
};

class TrackerAction : public solver::DiscretizedPoint {
    friend class TrackerTextSerializer;
  public:
    TrackerAction(ActionType actionType);
    TrackerAction(long code);

    virtual ~TrackerAction() = default;
    _NO_COPY_OR_MOVE(TrackerAction);

    std::unique_ptr<solver::Action> copy() const override;
    double distanceTo(solver::Action const &otherAction) const override;
    void print(std::ostream &os) const override;

    long getBinNumber() const override;
    ActionType getActionType() const;
  private:
    ActionType actionType_;
};
} /* namespace tracker */

#endif /* TRACKER_ACTION_HPP_ */
