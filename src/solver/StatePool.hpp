#ifndef SOLVER_STATEPOOL_HPP_
#define SOLVER_STATEPOOL_HPP_

#include <cstddef>                      // for size_t

#include <map>                          // for multimap
#include <memory>                       // for unique_ptr
#include <unordered_map>                // for unordered_map
#include <unordered_set>                // for unordered_set
#include <vector>                       // for vector

#include "ChangeFlags.hpp"               // for ChangeFlags
#include "State.hpp"                    // for State, operator==
#include "StateInfo.hpp"                // for StateInfo

namespace solver {
class StateIndex;

class StatePool {
    friend class Solver;
  public:
    struct Hash {
        std::size_t operator()(State * const &state) const {
            return state->hash();
        }
    };
    struct EqualityTest {
        bool operator()(State * const &s1,
                State * const &s2) const {
            return *s1 == *s2;
        }
    };
    typedef std::unordered_map<State *, StateInfo *,
            Hash, EqualityTest> StateInfoMap;

    friend class TextSerializer;

    StatePool(unsigned long nSDim);
    ~StatePool();
    StatePool(StatePool const &) = delete;
    StatePool(StatePool &&) = delete;
    StatePool &operator=(StatePool const &) = delete;
    StatePool &operator=(StatePool &&) = delete;

    void reset();

    StateInfo *getInfo(State *state);
    StateInfo *getInfoById(long stId);

    StateInfo *add(std::unique_ptr<StateInfo> stateInfo);
    StateInfo *createOrGetInfo(std::unique_ptr<State> state);

    StateIndex *getStateIndex();
    void addToStateIndex(StateInfo *stateInfo);

    void resetChangeFlags(StateInfo *stateInfo);
    void setChangeFlags(StateInfo *stateInfo, ChangeFlags flags);
    void resetAffectedStates();
    std::unordered_set<StateInfo *> getAffectedStates();

  private:
    unsigned long nSDim_;
    StateInfoMap stateInfoMap_;
    std::vector<std::unique_ptr<StateInfo>> statesByIndex_;
    std::unique_ptr<StateIndex> stateIndex_;

    std::unordered_set<StateInfo *> changedStates_;
};
} /* namespace solver */

#endif /* SOLVER_STATEPOOL_HPP_ */
