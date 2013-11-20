#ifndef STATEPOOL_H
#define STATEPOOL_H

#include <fstream>
#include <ostream>
#include <set>
#include <map>
#include <vector>

#include "State.h"
#include "Model.h"

struct CompStVals {
    bool operator()(const State* s1, const State* s2) const {
        StateVals s1Vals, s2Vals;
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
    bool operator()(const State* s1, const State* s2) const {
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

    void readStates(std::ifstream &inFile, Model *model);
    State* add(StateVals &sVals);
    State* getStPtr(long stId);
    void identifyAffectedStates(StateVals &lowLeft, StateVals &upRight,
            Change chType, std::set<State*> &affectedSt);
    //void getPosNNBelNode(State *s, double distTh, std::set<BeliefNode*> &res);
    void write(std::ostream &os);

private:
    long nStates, nSDim;
    std::set<State*, CompStVals> allStates;
    std::vector<State*> allStatesIdx;
    //std::vector< std::multimap<double, std::set<State*, CompStVals>::iterator> > stStruct;
    std::vector<std::multimap<double, State*> > stStruct;

};
#endif
