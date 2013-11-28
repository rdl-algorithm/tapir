#ifndef HISTORYENTRY_HPP
#define HISTORYENTRY_HPP

#include "Observation.hpp"
class BeliefNode;
class StateWrapper;

class HistoryEntry {
public:
    friend class BeliefNode;
    friend class HistorySequence;
    friend class Solver;
    friend class TextSerializer;

    HistoryEntry();
    HistoryEntry(StateWrapper *st);
    HistoryEntry(StateWrapper *st, long seqId, long entryId);
    ~HistoryEntry() = default;
    HistoryEntry(const HistoryEntry&) = delete;
    HistoryEntry(HistoryEntry&) = delete;
    HistoryEntry &operator=(const HistoryEntry&) = delete;
    HistoryEntry &operator=(HistoryEntry&) = delete;

    void setBelNode(BeliefNode *bel);

    void setSeqId(long seqId_) {
        seqId = seqId_;
    }
    void setNxt(long actId_, Observation &obs_) {
        actId = actId_, obs = obs_;
    }
    BeliefNode* getPartOfBelNode() {
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
    StateWrapper *st;
    bool hasBeenBackup;
    long seqId, entryId;
    double disc; // Net discount factor for the immediate reward
    double rew; // Non-discounted immediate reward
    double qVal; // Discounted total reward
    long actId;
    Observation obs;

    BeliefNode* partOfBelNode;
};

#endif /* HISTORYENTRY_HPP */
