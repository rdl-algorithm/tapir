#ifndef SOLVER_STATEINFO_HPP_
#define SOLVER_STATEINFO_HPP_

#include <memory>                       // for unique_ptr
#include <unordered_set>                          // for seteset(state.copy());
#include <vector>                       // for vector

#include "changes/ChangeFlags.hpp"               // for ChangeFlags
#include "abstract-problem/State.hpp"

#include "global.hpp"

namespace solver {
class BeliefNode;
class HistoryEntry;

class StateInfo {
  public:
    friend class Solver;
    friend class StatePool;
    friend class TextSerializer;

    /** Constructs a StateInfo with no associated state!! */
    StateInfo();
    /** Constructs a StateInfo to manage the given state. */
    StateInfo(std::unique_ptr<State> state);
    /** Constructs a StateInfo to manage the given state. */
    StateInfo(State const &state);

    /** Default destructor. */
    ~StateInfo();
    _NO_COPY_OR_MOVE(StateInfo);

    /* ----------------- History entry registration  ----------------- */
    /** Registers a history entry as containing this state. */
    void addHistoryEntry(HistoryEntry *entry);
    /** Deregisters a history entry that no longer contains this state. */
    void removeHistoryEntry(HistoryEntry *entry);

    /* ---------------------- Simple setters  ---------------------- */
    /** Sets the ID of this state to the given ID. */
    void setId(long id);
    /* ---------------------- Simple getters  ---------------------- */
    /** Returns the ID of this state. */
    long getId() const;
    /** Returns the state held by this StateInfo. */
    State const *getState() const;

  private:
    /* ---------------------- Model change handling  ---------------------- */
    void resetChangeFlags();
    void setChangeFlags(ChangeFlags flags);

    std::unique_ptr<State const> state_;
    long id_;

    std::unordered_set<HistoryEntry *> usedInHistoryEntries_;

    ChangeFlags changeFlags_;
};
} /* namespace solver */

#endif /* SOLVER_STATEINFO_HPP_ */
