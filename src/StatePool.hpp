#ifndef STATEPOOL_HPP
#define STATEPOOL_HPP

#include <iosfwd>
#include <map>
#include <set>
#include <vector>

#include "ChangeType.hpp"
#include "State.hpp"
#include "StateWrapper.hpp"
class Model;

struct CompStVals {
    bool operator()(const StateWrapper* s1, const StateWrapper* s2) const {
        State s1Vals, s2Vals;
        s1->getVals(s1Vals);
        s2->getVals(s2Vals);
        //cout << "s1Size: " << s1Vals.size() << " " << s2Vals.size() << endl;
        //cout << "compare: ( " << s1Vals[0] << " " << s1Vals[1] << " ) ( " << s2Vals[0] << " " << s2Vals[1] << ")\n";
        std::vector<double>::iterator it1, it2;
        for (it1 = s1Vals.begin(), it2 = s2Vals.begin(); it1 != s1Vals.end();
                it1++, it2++) {
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
    friend class Solver;

    StatePool();
    ~StatePool();
    StatePool(const StatePool&) = delete;
    StatePool(StatePool&) = delete;
    StatePool &operator=(const StatePool&) = delete;
    StatePool &operator=(StatePool&) = delete;

    void readStates(std::ifstream &inFile, Model *model);
    StateWrapper* add(State &sVals);
    StateWrapper* getStPtr(long stId);
    void identifyAffectedStates(State &lowLeft, State &upRight,
            ChangeType chType, std::set<StateWrapper*> &affectedSt);
    //void getPosNNBelNode(State *s, double distTh, std::set<BeliefNode*> &res);
    void write(std::ostream &os);

private:
    long nStates, nSDim;
    std::set<StateWrapper*, CompStVals> allStates;
    std::vector<StateWrapper*> allStatesIdx;
    //std::vector< std::multimap<double, std::set<State*, CompStVals>::iterator> > stStruct;
    std::vector<std::multimap<double, StateWrapper*> > stStruct;

};

#endif /* STATEPOOL_HPP */
