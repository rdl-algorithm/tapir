#ifndef HISTORYENTRY_HPP
#define HISTORYENTRY_HPP


#include <memory>                       // for unique_ptr

#include "defs.hpp"                     // for make_unique
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

    void setBelNode(BeliefNode *bel);

    void setSeqId(long seqId) {
        this->seqId = seqId;
    }

    void setNext(Action const &action, Observation const &obs) {
        this->action = action;
        this->obs = obs;
    }

    BeliefNode *getPartOfBelNode() {
        return partOfBelNode;
    }


    State *getState();

    StateInfo *getStateInfo() {
        return stateInfo;
    }

    long getId() {
        return entryId;
    }
    long getSeqId() {
        return seqId;
    }
    Action getAction() {
        return action;
    }

  private:
    HistoryEntry();
    StateInfo *stateInfo;
    bool hasBeenBackup;
    long seqId, entryId;
    double discount; // Net discount factor for the immediate reward
    double immediateReward; // Non-discounted immediate reward
    double qVal; // Discounted total reward
    Action action;
    Observation obs;

    BeliefNode *partOfBelNode;
};

#endif /* HISTORYENTRY_HPP */
