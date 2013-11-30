#ifndef BELIEFNODE_HPP
#define BELIEFNODE_HPP

#include <ctime>                        // for clock_t
#include <map>                          // for map
#include <queue>                        // for queue
#include <vector>                       // for vector
#include "Observation.hpp"              // for Observation
class ActionNode;
class HistoryEntry;

class BeliefNode {
public:
    friend class BeliefTree;
    friend class Solver;
    friend class TextSerializer;

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
    void enqueueChildren(std::queue<BeliefNode*> &res);

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
