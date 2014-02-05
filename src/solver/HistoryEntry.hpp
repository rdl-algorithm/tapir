#ifndef SOLVER_HISTORYENTRY_HPP_
#define SOLVER_HISTORYENTRY_HPP_

#include "Action.hpp"                   // for Action
#include "ChangeFlags.hpp"              // for ChangeFlags
#include "Observation.hpp"              // for Observation

namespace solver {
class BeliefNode;
class HistorySequence;
class State;
class StateInfo;

class HistoryEntry {
  public:
    friend class Histories;
    friend class HistorySequence;
    friend class Solver;
    friend class TextSerializer;
    friend class HistoryEntryComparator;

    /** Constructs a new history entry, without a state!! */
    HistoryEntry();
    /** Constructs a new history entry with the given cumulative discount,
     * owning sequence, and entry ID.
     */
    HistoryEntry(double discount, HistorySequence *owningSequence,
            long entryId);
    /** Constructs a new history entry with the given state info,
     *  cumulative discount, owning sequence, and entry ID.
     */
    HistoryEntry(StateInfo *stateInfo, double discount,
            HistorySequence *owningSequence, long entryId);

    /** Destroys this HistoryEntry. */
    ~HistoryEntry();

    // Copying and moving disallowed!
    HistoryEntry(HistoryEntry const &) = delete;
    HistoryEntry(HistoryEntry &&) = delete;
    HistoryEntry &operator=(HistoryEntry const &) = delete;
    HistoryEntry &operator=(HistoryEntry &&) = delete;


    /** Registers this history entry as a particle of the given belief node.
     * A value of nullptr will deregister this particle from that node.
     */
    void registerNode(BeliefNode *node);
    /** Registers this history entry as one of the particles that contains
     * the given state.
     * A value of nullptr will deregister this particle from that state.
     */
    void registerState(StateInfo *info);
    /** Registers this history entry with the given belief node,
     * and updates the StateInfo to be informed of its usage in the given
     * belief node and history entry.
     */

    /** Returns the state associated with this history entry. */
    State *getState();

    /**  Resets the changes that apply to this history entry. */
    void resetChangeFlags();
    /** Sets the given flags for this history entry. */
    void setChangeFlags(ChangeFlags flags);

  private:
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
    /** The id of the specific entry within the sequence. */
    long entryId_;
    /** Net discount factor, as applies to the immediate reward. */
    double discount_;
    /** Non-discounted immediate reward. */
    double immediateReward_;
    /** Total discounted reward. */
    double totalDiscountedReward_;

    /** The history sequence that owns this entry. */
    HistorySequence *owningSequence_;
    /** The belief node this entry belong to. */
    BeliefNode *owningBeliefNode_;

    /** The flags associated with current POMDP model updates. */
    ChangeFlags changeFlags_;
};
} /* namespace solver */

#endif /* SOLVER_HISTORYENTRY_HPP_ */
