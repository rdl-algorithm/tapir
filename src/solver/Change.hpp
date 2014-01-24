#ifndef SOLVER_CHANGE_HPP_
#define SOLVER_CHANGE_HPP_

#include "ChangeType.hpp"
#include "statesets.hpp"

namespace solver {
class StateIndex;

class Change {
  public:
    Change() = default;
    virtual ~Change() = default;

    void markAffectedStates(StateIndex *index);

    StateInfoSet getAffectedStates(StateIndex *index);
};

} /* namespace solver */

#endif /* SOLVER_CHANGE_HPP_ */
