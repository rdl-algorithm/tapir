#ifndef SOLVER_STATEINFO_HPP_
#define SOLVER_STATEINFO_HPP_

#include <memory>                       // for unique_ptr
#include <unordered_set>                          // for set
#include <vector>                       // for vector

#include "ChangeFlags.hpp"               // for ChangeFlags

namespace solver {
class BeliefNode;
class HistoryEntry;
class State;

class StateInfo {
  public:
    friend class TextSerializer;
    friend class StatePool;
    friend class Solver;

    /** Constructs a StateInfo with no associated state!! */
    StateInfo();
    /** Constructs a StateInfo to manage the given state. */
    StateInfo(std::unique_ptr<State> state);

    /** Default destructor. */
    ~StateInfo();
    // Copying and moving is disallowed.
    StateInfo(StateInfo const &) = delete;
    StateInfo(StateInfo &&) = delete;
    StateInfo &operator=(StateInfo const &) = delete;
    StateInfo &operator=(StateInfo &&) = delete;

    /** Returns the ID of this state. */
    long getId();
    /** Sets the ID of this state using the static counter. */
    void setId();

    /** Registers a history entry as containing this state. */
    void addHistoryEntry(HistoryEntry *entry);
    /** Deregisters a history entry that no longer contains this state. */
    void removeHistoryEntry(HistoryEntry *entry);

    /** Returns the state held by this StateInfo. */
    State *getState() const {
        return state_.get();
    }

  private:

    void resetChangeFlags();
    void setChangeFlags(ChangeFlags flags);

    static long currId;

    std::unique_ptr<State> state_;
    long id_;

    std::unordered_set<HistoryEntry *> usedInHistoryEntries_;

    ChangeFlags changeFlags_;
};
} /* namespace solver */

#endif /* SOLVER_STATEINFO_HPP_ */
