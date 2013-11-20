#ifndef STATE_H
#define STATE_H

#include <ostream>
#include <string>
#include <vector>
#include <set>

#include "Model.h"

class BeliefNode;
class HistoryEntry;

class State {
    public:
        friend class StatePool;
        friend class Solver;

        double distUse;

        State(StateVals &s_);
        State(std::string &str, long nStVars);
        ~State();

        void setId();
        void addInfo(HistoryEntry *h, BeliefNode *b);
        void addInfo(HistoryEntry *h);
        void addInfo(BeliefNode *b);
        double distL1(State *st);

        void delUsedInHistEntry(HistoryEntry *toBeDeleted);

        void write(std::ostream &os);
        void writeln(std::ostream &os);

        void getVals(StateVals &res) const { res = s; }
        long getId() const { return id; }

    private:
        static long currId;

        long id;
        StateVals s;
        std::vector<HistoryEntry*> usedInHistEntries;
        std::set<BeliefNode*> usedInBelNode;

        ChType chType;
};
#endif
