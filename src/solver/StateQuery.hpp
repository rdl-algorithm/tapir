#ifndef STATEQUERY_HPP_
#define STATEQUERY_HPP_

#include <unordered_set>

#include "StateSets.hpp"

class StateInfo;

namespace solver {
class StateQuery {
  public:
    StateQuery() = default;
    virtual ~StateQuery() = default;
    StateQuery(StateQuery const &) = delete;
    StateQuery(StateQuery &&) = delete;
    virtual StateQuery &operator=(StateQuery const &) = delete;
    virtual StateQuery &operator=(StateQuery &&) = delete;

    virtual StateInfoSet getStates() = 0;
    virtual void clearStates() = 0;
};
} /* namespace solver */

#endif /* STATEQUERY_HPP_ */
