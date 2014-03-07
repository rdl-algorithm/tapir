#ifndef TAG_ACTION_HPP_
#define TAG_ACTION_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/EnumeratedPoint.hpp"             // for EnumeratedPoint

namespace tag {
enum class ActionType : long {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
    TAG = 4,
};

class TagAction : public solver::EnumeratedPoint {
    friend class TagTextSerializer;
  public:
    TagAction(ActionType actionType);
    TagAction(long code);

    virtual ~TagAction() = default;
    _NO_COPY_OR_MOVE(TagAction);

    std::unique_ptr<solver::Action> copy() const override;
    double distanceTo(solver::Action const &otherAction) const override;
    void print(std::ostream &os) const override;

    long getCode() const override;

    ActionType getActionType() const;
  private:
    ActionType actionType_;
};
} /* namespace tag */

#endif /* TAG_ACTION_HPP_ */
