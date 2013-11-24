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

    HistoryEntry(StateWrapper *st, unsigned long entryId);
    HistoryEntry(StateWrapper *st, unsigned long seqId, unsigned long entryId);
    HistoryEntry(unsigned long seqId, unsigned long entryId, StateWrapper *st,
            std::stringstream &sstr);
    void initialise();
    ~HistoryEntry();

    void setBelNode(BeliefNode *bel);
    //void prepareDel();

    void write(std::ostream &os);
    void writeln(std::ostream &os);
    void writeSt(std::ostream &os);

    inline void setSeqId(long seqId_) {
        seqId = seqId_;
    }
    inline void setNxt(long actId_, ObsVals &obs_) {
        actId = actId_, obs = obs_;
    }
    inline BeliefNode* getPartOfBelNode() {
        return partOfBelNode;
    }
    inline long getId() {
        return entryId;
    }
    inline long getSeqId() {
        return seqId;
    }
    inline long getActId() {
        return actId;
    }

private:
    StateWrapper *st;
    bool hasBeenBackup;
    unsigned long seqId, entryId;
    double disc, rew, qVal; // disc: discount factor for the immediate reward, rew: not discounted immediate reward, qVal: discounted total reward.
    long actId;
    ObsVals obs;

    BeliefNode* partOfBelNode;

};
#endif
