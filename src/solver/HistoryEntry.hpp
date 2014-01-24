#ifndef SOLVER_HISTORYENTRY_HPP_
#define SOLVER_HISTORYENTRY_HPP_

#include "Action.hpp"                   // for Action
#include "Observation.hpp"              // for Observation

namespace solver {
class BeliefNode;
class State;
class StateInfo;

class HistoryEntry {
  public:
    friend class HistorySequence;
    friend class Solver;
    friend class TextSerializer;

    /** Constructs a new history entry with the given state, sequence ID, and
     * entry ID.
     */
    HistoryEntry(StateInfo *stateInfo, long seqId, long entryId);
    /** Constructs a new history entry with the given state. */
    HistoryEntry(StateInfo *stateInfo);

    /** Default destructor. */
    ~HistoryEntry() = default;

    /* Copying and moving is disallowed. */
    HistoryEntry(HistoryEntry const &) = delete;
    HistoryEntry(HistoryEntry &&) = delete;
    HistoryEntry &operator=(HistoryEntry const &) = delete;
    HistoryEntry &operator=(HistoryEntry &&) = delete;

    /** Returns the state associated with this history entry. */
    State *getState();

  private:
    HistoryEntry();
    /** The state information for this history entry. */
    StateInfo *stateInfo_;

    /** Action performed in this entry. */
    Action action_;
    /** Observation received in this entry. */
    Observation observation_;

    /** True iff this entry has been processed in a Bellman backup,
     * false otherwise.
     */
    bool hasBeenBackedUp_;
    /** The id of the sequence. */
    long seqId_;
    /** The id of the specific entry within the sequence. */
    long entryId_;
    /** Net discount factor, as applies to the immediate reward. */
    double discount_;
    /** Non-discounted immediate reward. */
    double immediateReward_;
    /** Total discounted reward. */
    double totalDiscountedReward_;

    /** The belief node this entry belong to. */
    BeliefNode *owningBeliefNode_;
};
} /* namespace solver */

#endif /* SOLVER_HISTORYENTRY_HPP_ */
