#ifndef SOLVER_CHANGE_HPP_
#define SOLVER_CHANGE_HPP_

#include <unordered_set>

#include "ChangeType.hpp"

namespace solver {
class StateIndex;

class Change {
  public:
    Change() = default;
    virtual ~Change() = default;

    void markAffectedStates(StateIndex *index);

    std::unordered_set<StateInfo*> getAffectedStates(StateIndex *index);
};

} /* namespace solver */

#endif /* SOLVER_CHANGE_HPP_ */
