/** @file HomecareAction.hpp
 *
 * Defines the HomecareAction class, which represents an action for the Homecare problem, and also the
 * ActionType enumeration, which enumerates the different types of actions for Homecare.
 */
#ifndef HOMECARE_ACTION_HPP_
#define HOMECARE_ACTION_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/DiscretizedPoint.hpp"             // for DiscretizedPoint

namespace homecare {

/** An enumeration of all the available actions in the Homecare PODMP. */
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

/** A class representing an action in the Homecare POMDP.
 *
 * This class also implements solver::DiscretizedPoint so that the solver can use a simplistic
 * enumerated action mapping approach (EnumeratedActionPool) to store the available actions from
 * each belief node.
 */
class HomecareAction : public solver::DiscretizedPoint {
    friend class HomecareTextSerializer;
  public:
    /** Constructs a new action from the given ActionType. */
    HomecareAction(ActionType actionType);
    /** Constructs a new action from the given integer code. */
    HomecareAction(long code);

    virtual ~HomecareAction() = default;
    _NO_COPY_OR_MOVE(HomecareAction);

    std::unique_ptr<solver::Action> copy() const override;
    double distanceTo(solver::Action const &otherAction) const override;
    void print(std::ostream &os) const override;

    long getBinNumber() const override;
    /** Returns the ActionType of this action. */
    ActionType getActionType() const;
  private:
    /** The ActionType for this action in the Homecare POMDP. */
    ActionType actionType_;
};
} /* namespace homecare */

#endif /* HOMECARE_ACTION_HPP_ */
