/** @file TagAction.hpp
 *
 * Defines the TagAction class, which represents an action for the Tag problem, and also the
 * ActionType enumeration, which enumerates the different types of actions for Tag.
 */
#ifndef TAG_ACTION_HPP_
#define TAG_ACTION_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/DiscretizedPoint.hpp"             // for DiscretizedPoint

namespace tag {

/** An enumeration of all the available actions in the Tag PODMP. */
enum class ActionType : long {
    /** The action to move north. */
    NORTH = 0,
    /** The action to move east. */
    EAST = 1,
    /** The action to move south. */
    SOUTH = 2,
    /** The action to move west. */
    WEST = 3,
    /** The action to attempt to tag the opponent. */
    TAG = 4,
};

/** A class representing an action in the Tag POMDP.
 *
 * This class also implements solver::DiscretizedPoint so that the solver can use a simplistic
 * enumerated action mapping approach (EnumeratedActionPool) to store the available actions from
 * each belief node.
 */
class TagAction : public solver::DiscretizedPoint {
    friend class TagTextSerializer;
  public:
    /** Constructs a new action from the given ActionType. */
    TagAction(ActionType actionType);
    /** Constructs a new action from the given integer code. */
    TagAction(long code);

    virtual ~TagAction() = default;
    _NO_COPY_OR_MOVE(TagAction);

    std::unique_ptr<solver::Action> copy() const override;
    double distanceTo(solver::Action const &otherAction) const override;
    void print(std::ostream &os) const override;

    long getBinNumber() const override;
    /** Returns the ActionType of this action. */
    ActionType getActionType() const;

  private:
    /** The ActionType for this action in the Tag POMDP. */
    ActionType actionType_;
};
} /* namespace tag */

#endif /* TAG_ACTION_HPP_ */
