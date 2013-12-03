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
    typedef std::unordered_set<StateInfo*, StateInfo::StateHash, StateInfo::SameState> SetType;

    friend class TextSerializer;

    StatePool();
    ~StatePool();
    StatePool(const StatePool&) = delete;
    StatePool(StatePool&&) = delete;
    StatePool &operator=(const StatePool&) = delete;
    StatePool &operator=(StatePool&&) = delete;

    void reset();
    StateInfo *add(std::unique_ptr<State> state);
    StateInfo *getStateById(long stId);
    void identifyAffectedStates(State &lowLeft, State &upRight,
            ChangeType chType, std::set<StateInfo*> &affectedSt);

private:
    long nStates, nSDim;
    SetType allStates;
    std::vector<StateInfo*> allStatesIdx;
    std::vector<std::multimap<double, StateInfo*> > stStruct;
};

#endif /* STATEPOOL_HPP */
