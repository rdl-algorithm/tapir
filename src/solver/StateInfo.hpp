#ifndef STATEINFO_HPP
#define STATEINFO_HPP

#include <memory>                       // for unique_ptr
#include <set>                          // for set
#include <vector>                       // for vector

#include "ChangeType.hpp"               // for ChangeType

class BeliefNode;
class HistoryEntry;
class State;

class StateInfo {
  public:
    friend class TextSerializer;
    friend class StatePool;
    friend class Solver;

    StateInfo(std::unique_ptr<State> state);
    ~StateInfo();
    StateInfo(StateInfo const &) = delete;
    StateInfo(StateInfo &&) = delete;
    StateInfo &operator=(StateInfo const &) = delete;
    StateInfo &operator=(StateInfo &&) = delete;

    void setId();
    void addHistoryEntry(HistoryEntry *h);
    void addBeliefNode(BeliefNode *b);
    void delUsedInHistEntry(HistoryEntry *toBeDeleted);

    State *getState() const {
        return state.get();
    }

    long getId() const {
        return id;
    }
  private:
    StateInfo();

    std::unique_ptr<State> state;
    static long currId;
    long id;

    std::vector<HistoryEntry *> usedInHistoryEntries;
    std::set<BeliefNode *> usedInBeliefNodes;

    ChangeType chType;
};

#endif /* STATEINFO_HPP */
