#ifndef STATEQUERY_HPP_
#define STATEQUERY_HPP_

#include <unordered_set>

#include "StateSets.hpp"

class StateInfo;

namespace solver {
class StateIndexQuery {
  public:
    StateIndexQuery() = default;
    virtual ~StateIndexQuery() = default;
    StateIndexQuery(StateIndexQuery const &) = delete;
    StateIndexQuery(StateIndexQuery &&) = delete;
    virtual StateIndexQuery &operator=(StateIndexQuery const &) = delete;
    virtual StateIndexQuery &operator=(StateIndexQuery &&) = delete;

    virtual StateInfoSet getStates() = 0;
    virtual void clearStates() = 0;
};
} /* namespace solver */

#endif /* STATEQUERY_HPP_ */
