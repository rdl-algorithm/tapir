#ifndef STATEWRAPPER_HPP
#define STATEWRAPPER_HPP

#include <iosfwd>
#include <set>
#include <string>
#include <vector>

#include "ChangeType.hpp"
#include "State.hpp"
class BeliefNode;
class HistoryEntry;

class StateWrapper {
public:
    friend class TextSerializer;
    friend class StatePool;
    friend class Solver;

    StateWrapper(State &s);
    StateWrapper(std::string &str, long nStVars);
    ~StateWrapper() = default;
    StateWrapper(const StateWrapper&) = delete;
    StateWrapper(StateWrapper&) = delete;
    StateWrapper &operator=(const StateWrapper&) = delete;
    StateWrapper &operator=(StateWrapper&) = delete;

    void setId();
    void addInfo(HistoryEntry *h);
    void addInfo(BeliefNode *b);
    double distL1(StateWrapper *st);

    void delUsedInHistEntry(HistoryEntry *toBeDeleted);

    void write(std::ostream &os);
    void writeln(std::ostream &os);

    void getVals(State &res) const {
        res = s;
    }
    long getId() const {
        return id;
    }

private:
    StateWrapper();

    State s;
    static long currId;
    long id;

    std::vector<HistoryEntry*> usedInHistoryEntries;
    std::set<BeliefNode*> usedInBeliefNodes;

    ChangeType chType;
};

#endif /* STATEWRAPPER_HPP */
