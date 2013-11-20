#ifndef BELIEFNODE_H
#define BELIEFNODE_H

#include <string>
#include <ostream>
#include <vector>
#include <queue>
#include <map>

#include <ctime>

#include "Histories.h"
#include "HistoryEntry.h"
#include "ActionNode.h"

//using namespace cv;

class BeliefNode {
public:
    friend class BeliefTree;
    friend class Solver;

    bool distChecked;
    static long maxParticles;
    static long nStVars;

    BeliefNode();
    BeliefNode(long id_);
    ~BeliefNode();

    void set(std::stringstream &sstr, Histories *allHist);
    void setAct(std::string str, std::vector<BeliefNode*> &tmpNodes);
    long getUCBAct();
    long getBestAct();
    void add(HistoryEntry *newHistEntry);
    BeliefNode* addChild(long actIdx, ObsVals &obs, HistoryEntry* nxtHistEntry);
    BeliefNode* addChild(long actIdx, ObsVals &obs);
    HistoryEntry* sampleAParticle();
    void updateVal(long actIdx, double newVal);
    void updateVal(long actIdx, double prevVal, double newVal, bool cutPart);
    double distL1Independent(BeliefNode *b);

    BeliefNode* getChild(long actIdx, ObsVals &obs);
    void getChildren(std::queue<BeliefNode*> &res);
    void write(std::ostream &os);
    void writeNGetChildren(std::ostream &os, std::queue<BeliefNode*> &res);
    void writeStParticles(std::ostream &os);

    long getNxtActToTry();
    void calcBestVal();

    inline long getId() {
        return id;
    }
    inline long getNParticles() {
        return nParticles;
    }
    inline long getNActChildren() {
        return nActChildren;
    }

private:
    static long currId;
    static double exploreParam;
    static std::clock_t startTime;

    long id, nParticles, nActChildren, nxtActToTry;
    double bestAvgQVal;
    long bestAct;

    double tLastAddedParticle, tNNComp, tEmdSig;
    BeliefNode *nnBel;

    std::vector<HistoryEntry*> particles;
    std::map<long, ActionNode*> actChildren, invalidActChildren;

};
#endif
