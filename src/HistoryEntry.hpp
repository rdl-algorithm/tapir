#ifndef HISTORYENTRY_HPP
#define HISTORYENTRY_HPP

#include "Observation.hpp"              // for Observation
class BeliefNode;
class StateInfo;


class HistoryEntry {
  public:
    friend class BeliefNode;
    friend class HistorySequence;
    friend class Solver;
    friend class TextSerializer;

    HistoryEntry(StateInfo *stateInfo);
    HistoryEntry(StateInfo *stateInfo, long seqId, long entryId);
    ~HistoryEntry() = default;
    HistoryEntry(HistoryEntry const &) = delete;
    HistoryEntry(HistoryEntry &&) = delete;
    HistoryEntry &operator=(HistoryEntry const &) = delete;
    HistoryEntry &operator=(HistoryEntry &&) = delete;

    void setBelNode(BeliefNode *bel);

    void setSeqId(long seqId_) {
        seqId = seqId_;
    }
    void setNxt(long actId_, Observation &obs_) {
        actId = actId_, obs = obs_;
    }
    BeliefNode *getPartOfBelNode() {
        return partOfBelNode;
    }

    long getId() {
        return entryId;
    }
    long getSeqId() {
        return seqId;
    }
    long getActId() {
        return actId;
    }

  private:
    HistoryEntry();
    StateInfo *stateInfo;
    bool hasBeenBackup;
    long seqId, entryId;
    double discount; // Net discount factor for the immediate reward
    double immediateReward; // Non-discounted immediate reward
    double qVal; // Discounted total reward
    long actId;
    Observation obs;

    BeliefNode *partOfBelNode;
};

#endif /* HISTORYENTRY_HPP */
