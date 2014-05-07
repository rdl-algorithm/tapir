#ifndef SOLVER_HISTORYENTRY_HPP_
#define SOLVER_HISTORYENTRY_HPP_

#include <memory>

#include "abstract-problem/Action.hpp"                   // for Action
#include "abstract-problem/Observation.hpp"              // for Observation
#include "abstract-problem/State.hpp"

#include "abstract-problem/TransitionParameters.hpp"
#include "changes/ChangeFlags.hpp"              // for ChangeFlags

#include "global.hpp"

namespace solver {
class BeliefNode;
class HistorySequence;
class StateInfo;

class HistoryEntry {
  public:
    friend class AbstractBackpropagationStrategy;
    friend class AbstractSearchInstance;
    friend class DefaultHistoryCorrector;
    friend class HistorySequence;
    friend class Solver;
    friend class TextSerializer;

    /** Constructs a new history entry, without a state!! */
    HistoryEntry();
    /** Constructs a new history entry with the given cumulative discount,
     * owning sequence, and entry ID.
     */
    HistoryEntry(HistorySequence *owningSequence, long entryId);
    /** Constructs a new history entry with the given state info,
     *  cumulative discount, owning sequence, and entry ID.
     */
    HistoryEntry(StateInfo *stateInfo, HistorySequence *owningSequence,
            long entryId);
    /** Destroys this HistoryEntry. */
    ~HistoryEntry();
    _NO_COPY_OR_MOVE(HistoryEntry);

    /* ----------------- Simple getters ------------------- */
    /** Returns the id of this entry (0 = first entry in the sequence). */
    long getId() const;
    /** Returns the immediate reward for this entry. */
    double getReward() const;
    /** Returns the cumulative discounted reward, starting at this entry. */
    double getCumulativeReward() const;
    /** Returns the state associated with this history entry. */
    State const *getState() const;
    /** Returns the action associated with this history entry. */
    Action const *getAction() const;
    /** Returns the observation associated with this history entry. */
    Observation const *getObservation() const;
    /** Returns the transition parameters associated with this
     * history entry.
     */
    TransitionParameters const *getTransitionParameters() const;
    /** Returns the belief node associated with this history entry. */
    BeliefNode *getAssociatedBeliefNode() const;

    /* -------------- Registration methods ---------------- */
    /** Returns true iff this entry is already registered as a particle. */
    bool isRegisteredAsParticle() const;

private:
    /* ----------------- Change flagging ------------------- */
    /**  Resets the changes that apply to this history entry. */
    void resetChangeFlags();
    /** Sets the given flags for this history entry. */
    void setChangeFlags(ChangeFlags flags);

    /* -------------- Registration methods ---------------- */
    /** Registers this history entry as a particle of the given belief node.
     * A value of nullptr will deregister this particle from that node.
     */
    void registerNode(BeliefNode *node);
    /** Registers this history entry as one of the particles that contains
     * the given state.
     * A value of nullptr will deregister this particle from that state.
     */
    void registerState(StateInfo *info);

  private:
    /** The state information for this history entry. */
    StateInfo *stateInfo_;
    /** Action performed in this entry. */
    std::unique_ptr<Action> action_;
    /** Extra information about the transition, if any is needed. */
    std::unique_ptr<TransitionParameters> transitionParameters_;
    /** Observation received in this entry. */
    std::unique_ptr<Observation> observation_;

    /** True iff this entry has been backpropagated, and false otherwise. */
    bool hasBeenBackedUp_;
    /** The id of the specific entry within the sequence. */
    long entryId_;
    /** Non-discounted reward. */
    double reward_;
    /** Cumulative discounted reward, starting from this action. */
    double rewardFromHere_;

    /** The history sequence that owns this entry. */
    HistorySequence *owningSequence_;
    /** The belief node this entry is associated with. */
    BeliefNode *associatedBeliefNode_;
    /** Whether a belief node has this entry registered as a particle. */
    bool isRegisteredAsParticle_;

    /** The flags associated with current POMDP model updates. */
    ChangeFlags changeFlags_;
};
} /* namespace solver */

#endif /* SOLVER_HISTORYENTRY_HPP_ */
