#ifndef HOMECARE_ACTION_HPP_
#define HOMECARE_ACTION_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/DiscretizedPoint.hpp"             // for DiscretizedPoint

namespace homecare {
enum class ActionType : long {
    NORTH = 0,
    NORTH_EAST = 1,
    EAST = 2,
    SOUTH_EAST = 3,
    SOUTH = 4,
    SOUTH_WEST = 5,
    WEST = 6,
    NORTH_WEST = 7,
    WAIT = 8,
};

class HomecareAction : public solver::DiscretizedPoint {
    friend class HomecareTextSerializer;
  public:
    HomecareAction(ActionType actionType);
    HomecareAction(long code);

    virtual ~HomecareAction() = default;
    _NO_COPY_OR_MOVE(HomecareAction);

    std::unique_ptr<solver::Action> copy() const override;
    double distanceTo(solver::Action const &otherAction) const override;
    void print(std::ostream &os) const override;

    long getBinNumber() const override;
    ActionType getActionType() const;
  private:
    ActionType actionType_;
};
} /* namespace homecare */

#endif /* HOMECARE_ACTION_HPP_ */
