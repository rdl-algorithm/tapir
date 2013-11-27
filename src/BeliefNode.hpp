#ifndef BELIEFNODE_HPP
#define BELIEFNODE_HPP

#include <ctime>

#include <iosfwd>
#include <map>
#include <queue>
#include <string>
#include <vector>

#include "Observation.hpp"
class ActionNode;
class Histories;
class HistoryEntry;

class BeliefNode {
public:
    friend class BeliefTree;
    friend class Solver;

    bool distChecked;
    static long maxParticles;
    static long nStVars;

    BeliefNode();
    BeliefNode(long id);
    ~BeliefNode();
    BeliefNode(const BeliefNode&) = delete;
    BeliefNode(BeliefNode&) = delete;
    BeliefNode &operator=(const BeliefNode&) = delete;
    BeliefNode &operator=(BeliefNode&) = delete;

    void set(std::stringstream &sstr, Histories *allHist);
    void setAct(std::string str, std::vector<BeliefNode*> &tmpNodes);
    long getUCBAct();
    long getBestAct();
    void add(HistoryEntry *newHistEntry);
    BeliefNode* addChild(long actIdx, Observation &obs,
            HistoryEntry* nxtHistEntry);
    BeliefNode* addChild(long actIdx, Observation &obs);
    HistoryEntry* sampleAParticle();
    void updateVal(long actIdx, double newVal);
    void updateVal(long actIdx, double prevVal, double newVal, bool cutPart);
    double distL1Independent(BeliefNode *b);

    BeliefNode* getChild(long actIdx, Observation &obs);
    void getChildren(std::queue<BeliefNode*> &res);
    void write(std::ostream &os);
    void writeNGetChildren(std::ostream &os, std::queue<BeliefNode*> &res);
    void writeStParticles(std::ostream &os);

    long getNxtActToTry();
    void calcBestVal();

    long getId() {
        return id;
    }
    long getNParticles() {
        return nParticles;
    }
    long getNActChildren() {
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
    std::map<long, ActionNode*> actChildren;
};

#endif /* BELIEFNODE_HPP */
