#ifndef SOLVER_STATEINFO_HPP_
#define SOLVER_STATEINFO_HPP_

#include <memory>                       // for unique_ptr
#include <unordered_set>                          // for seteset(state.copy());
#include <vector>                       // for vector

#include "ChangeFlags.hpp"               // for ChangeFlags
#include "topology/State.hpp"

#include "global.hpp"

namespace solver {
class BeliefNode;
class HistoryEntry;

class StateInfo {
  public:
    friend class TextSerializer;
    friend class StatePool;
    friend class Solver;

    /** Constructs a StateInfo with no associated state!! */
    StateInfo();
    /** Constructs a StateInfo to manage the given state. */
    StateInfo(std::unique_ptr<State> state);
    /** Constructs a StateInfo to manage the given state. */
    StateInfo(State const &state);

    /** Default destructor. */
    ~StateInfo();
    _NO_COPY_OR_MOVE(StateInfo);

    /** Returns the ID of this state. */
    long getId() const;
    /** Sets the ID of this state using the static counter. */
    void setId();

    /** Registers a history entry as containing this state. */
    void addHistoryEntry(HistoryEntry *entry);
    /** Deregisters a history entry that no longer contains this state. */
    void removeHistoryEntry(HistoryEntry *entry);

    /** Returns the state held by this StateInfo. */
    State const *getState() const;

  private:
    void resetChangeFlags();
    void setChangeFlags(ChangeFlags flags);

    static long currId;

    std::unique_ptr<State const> state_;
    long id_;

    std::unordered_set<HistoryEntry *> usedInHistoryEntries_;

    ChangeFlags changeFlags_;
};
} /* namespace solver */

#endif /* SOLVER_STATEINFO_HPP_ */
