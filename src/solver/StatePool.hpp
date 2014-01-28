#ifndef SOLVER_STATEPOOL_HPP_
#define SOLVER_STATEPOOL_HPP_

#include <cstddef>                      // for size_t

#include <map>                          // for multimap
#include <memory>                       // for unique_ptr
#include <set>                          // for set
#include <unordered_set>                // for unordered_set
#include <vector>                       // for vector

#include "ChangeType.hpp"               // for ChangeType
#include "State.hpp"                    // for State, operator==
#include "StateInfo.hpp"                // for StateInfo

namespace solver {
class StateIndex;

class StatePool {
  public:
    struct Hash {
        std::size_t operator()(std::unique_ptr<StateInfo> const &stateInfo) const {
            return stateInfo->getState()->hash();
        }
    };
    struct EqualityTest {
        bool operator()(std::unique_ptr<StateInfo> const &s1,
                std::unique_ptr<StateInfo> const &s2) const {
            return *(s1->getState()) == *(s2->getState());
        }
    };
    typedef std::unordered_set<std::unique_ptr<StateInfo>, Hash,
            EqualityTest> StateInfoOwningSet;


    friend class TextSerializer;

    StatePool(unsigned long nSDim);
    ~StatePool();
    StatePool(StatePool const &) = delete;
    StatePool(StatePool &&) = delete;
    StatePool &operator=(StatePool const &) = delete;
    StatePool &operator=(StatePool &&) = delete;

    void reset();
    StateInfo *add(std::unique_ptr<StateInfo> stateInfo);
    StateInfo *add(std::unique_ptr<State> state);
    StateInfo *getStateById(long stId);

    StateIndex *getStateIndex();
    void addToStateIndex(StateInfo *stateInfo);

  private:
    unsigned long nSDim_;
    StateInfoOwningSet allStates_;
    std::vector<StateInfo *> statesByIndex_;
    std::unique_ptr<StateIndex> stateIndex_;
};
} /* namespace solver */

#endif /* SOLVER_STATEPOOL_HPP_ */
