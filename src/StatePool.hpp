#ifndef STATEPOOL_HPP
#define STATEPOOL_HPP

#include <map>                          // for multimap
#include <memory>                       // for unique_ptr
#include <unordered_set>                // for unordered_set
#include <vector>                       // for vector, vector<>::iterator
#include <utility>                      // for pair

#include "ChangeType.hpp"               // for ChangeType
#include "State.hpp"                    // for State
#include "StateInfo.hpp"                // for StateInfo
class StatePool {
public:
    struct StateInfoHash {
        size_t operator()(std::unique_ptr<StateInfo> const &stateInfo) const {
            return stateInfo->getState()->hash();
        }
    };
    struct SameStateInfo {
        bool operator()(std::unique_ptr<StateInfo> const &s1,
                std::unique_ptr<StateInfo> const &s2) const {
            return *(s1->getState()) == *(s2->getState());
        }
    };
    typedef std::unordered_set<std::unique_ptr<StateInfo>, StateInfoHash,
            SameStateInfo> StateInfoSet;

    friend class TextSerializer;

    StatePool();
    ~StatePool();
    StatePool(StatePool const &) = delete;
    StatePool(StatePool &&) = delete;
    StatePool &operator=(StatePool const &) = delete;
    StatePool &operator=(StatePool &&) = delete;

    void reset();
    StateInfo *add(std::unique_ptr<State> state);
    StateInfo *getStateById(long stId);
    void identifyAffectedStates(State &lowLeft, State &upRight,
                                ChangeType chType, std::set<StateInfo *> &affectedSt);

  private:
    long nSDim;
    StateInfoSet allStates;
    std::vector<StateInfo *> allStatesIdx;
    std::vector<std::multimap<double, StateInfo *> > stStruct;
};

#endif /* STATEPOOL_HPP */
