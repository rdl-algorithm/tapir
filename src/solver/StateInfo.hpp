#ifndef SOLVER_STATEINFO_HPP_
#define SOLVER_STATEINFO_HPP_

#include <memory>                       // for unique_ptr
#include <set>                          // for set
#include <vector>                       // for vector

#include "ChangeType.hpp"               // for ChangeType

namespace solver {
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

    long getId();
    void setId();

    void addHistoryEntry(HistoryEntry *entry);
    void removeHistoryEntry(HistoryEntry *entry);

    void addBeliefNode(BeliefNode *node);
    void removeBeliefNode(BeliefNode *node);

    State *getState() const {
        return state_.get();
    }

  private:
    StateInfo();

    static long currId;

    std::unique_ptr<State> state_;
    long id_;

    std::set<HistoryEntry *> usedInHistoryEntries_;
    std::set<BeliefNode *> usedInBeliefNodes_;

    ChangeType changeType_;
};
} /* namespace solver */

#endif /* SOLVER_STATEINFO_HPP_ */
