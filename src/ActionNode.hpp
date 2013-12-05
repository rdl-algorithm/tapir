#ifndef ACTIONNODE_HPP
#define ACTIONNODE_HPP

#include <queue>                        // for queue
#include <vector>                       // for vector

#include "Observation.hpp"              // for Observation
class BeliefNode;
class ObservationEdge;

class ActionNode {
  public:
    friend class BeliefNode;
    friend class TextSerializer;

    ActionNode(long actId, Observation &obs, BeliefNode *nxtBelNode);
    ActionNode(long actId, long nParticles, double qVal, double avgQVal);
    ~ActionNode();
    ActionNode(ActionNode const &) = delete;
    ActionNode(ActionNode &&) = delete;
    ActionNode &operator=(ActionNode const &) = delete;
    ActionNode &operator=(ActionNode &&) = delete;

    void updateQVal(double newVal);
    void updateQVal(double prevVal, double newVal, bool reduceParticles);
    bool isAct(long aIdx);
    void addChild(ObservationEdge *edge);
    void addChild(Observation &obs, BeliefNode *nxtBelNode);
    BeliefNode *getObsChild(Observation &obs);
    void enqueueChildren(std::queue<BeliefNode *> &res);

    double getQVal() {
        return qVal;
    }

  private:
    ActionNode();
    long actId, nParticles;
    double qVal, avgQVal;

    std::vector<ObservationEdge *> obsChildren;
};

#endif /* ACTIONNODE_HPP */
