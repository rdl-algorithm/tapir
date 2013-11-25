#ifndef HISTORYENTRY_H
#define HISTORYENTRY_H

#include <ostream>
#include <string>
#include <sstream>

#include "StateWrapper.h"
#include "Model.h"

class BeliefNode;

class HistoryEntry {
public:
    friend class HistorySequence;
    friend class BeliefNode;
    friend class Solver;

    HistoryEntry(StateWrapper *st);
    HistoryEntry(StateWrapper *st, long seqId, long entryId);
    HistoryEntry(StateWrapper *st, long seqId, long entryId,
            std::stringstream &sstr);
    ~HistoryEntry() = default;
    HistoryEntry(const HistoryEntry&) = delete;
    HistoryEntry(HistoryEntry&) = delete;
    HistoryEntry &operator=(const HistoryEntry&) = delete;
    HistoryEntry &operator=(HistoryEntry&) = delete;

    void setBelNode(BeliefNode *bel);
    //void prepareDel();

    void write(std::ostream &os);
    void writeln(std::ostream &os);
    void writeSt(std::ostream &os);

    void setSeqId(long seqId_) {
        seqId = seqId_;
    }
    void setNxt(long actId_, ObsVals &obs_) {
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
    ObsVals obs;

    BeliefNode* partOfBelNode;
};
#endif
