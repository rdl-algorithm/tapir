#ifndef HISTORYENTRY_HPP
#define HISTORYENTRY_HPP

#include "Action.hpp"                   // for Action
#include "Observation.hpp"              // for Observation

class BeliefNode;
class State;
class StateInfo;

class HistoryEntry {
  public:
    friend class HistorySequence;
    friend class Solver;
    friend class TextSerializer;

    HistoryEntry(StateInfo *stateInfo, long seqId, long entryId);
    HistoryEntry(StateInfo *stateInfo);

    ~HistoryEntry() = default;
    HistoryEntry(HistoryEntry const &) = delete;
    HistoryEntry(HistoryEntry &&) = delete;
    HistoryEntry &operator=(HistoryEntry const &) = delete;
    HistoryEntry &operator=(HistoryEntry &&) = delete;

    State *getState();

  private:
    HistoryEntry();
    /** The state information for this history entry. */
    StateInfo *stateInfo;

    /** Action performed in this entry. */
    Action action;
    /** Observation received in this entry. */
    Observation observation;

    /** True iff this entry has been processed in a Bellman backup,
     * false otherwise.
     */
    bool hasBeenBackedUp;
    /** The id of the sequence. */
    long seqId;
    /** The id of the specific entry within the sequence. */
    long entryId;
    /** Net discount factor, as applies to the immediate reward. */
    double discount;
    /** Non-discounted immediate reward. */
    double immediateReward;
    /** Discounted expected total reward. */
    double qVal;

    /** The belief node this entry belong to. */
    BeliefNode *owningBeliefNode;
};

#endif /* HISTORYENTRY_HPP */
