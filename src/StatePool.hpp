#ifndef STATEPOOL_HPP
#define STATEPOOL_HPP

#include <map>                          // for multimap
#include <set>                          // for set
#include <vector>                       // for vector, vector<>::iterator
#include "ChangeType.hpp"               // for ChangeType
#include "State.hpp"                    // for State
#include "StateWrapper.hpp"             // for StateWrapper
struct CompStVals {
    bool operator()(const StateWrapper* s1, const StateWrapper* s2) const {
        State state1, state2;
        s1->getVals(state1);
        s2->getVals(state2);
        //cout << "s1Size: " << s1Vals.size() << " " << s2Vals.size() << endl;
        //cout << "compare: ( " << s1Vals[0] << " " << s1Vals[1] << " ) ( " << s2Vals[0] << " " << s2Vals[1] << ")\n";
        std::vector<double>::iterator it1, it2;
        for (it1 = state1.vals.begin(), it2 = state2.vals.begin();
                it1 != state1.vals.end(); it1++, it2++) {
            //cout << "it1: " << *it1 << " it2: " << *it2 << " min: " << *it1 - *it2 << endl;
            if (*it1 - *it2 < 0) {
                return true;
            } else if (*it1 - *it2 > 0) {
                return false;
            }
        }
        return false;
    }
};

struct CompIdVals {
    bool operator()(const StateWrapper* s1, const StateWrapper* s2) const {
        if ((s1->getId() - s2->getId()) < 0) {
            return true;
        } else if ((s1->getId() - s2->getId()) > 0) {
            return false;
        }
        return false;
    }
};

class StatePool {
public:
    friend class TextSerializer;

    StatePool();
    ~StatePool();
    StatePool(const StatePool&) = delete;
    StatePool(StatePool&) = delete;
    StatePool &operator=(const StatePool&) = delete;
    StatePool &operator=(StatePool&) = delete;

    void reset();
    StateWrapper* add(State &sVals);
    StateWrapper* getStateById(long stId);
    void identifyAffectedStates(State &lowLeft, State &upRight,
            ChangeType chType, std::set<StateWrapper*> &affectedSt);

private:
    long nStates, nSDim;
    std::set<StateWrapper*, CompStVals> allStates;
    std::vector<StateWrapper*> allStatesIdx;
    std::vector<std::multimap<double, StateWrapper*> > stStruct;
};

#endif /* STATEPOOL_HPP */
