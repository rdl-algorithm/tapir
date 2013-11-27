#ifndef OBSERVATIONEDGE_HPP
#define OBSERVATIONEDGE_HPP

#include <ostream>
#include <queue>

#include "Observation.hpp"

class BeliefNode;

class ObservationEdge {
public:
    ObservationEdge();
    ObservationEdge(Observation &o, BeliefNode* nxtBelNode);

    ~ObservationEdge() = default;
    ObservationEdge(const ObservationEdge&) = delete;
    ObservationEdge(ObservationEdge&) = delete;
    ObservationEdge &operator=(const ObservationEdge&) = delete;
    ObservationEdge &operator=(ObservationEdge&) = delete;

    bool isObs(Observation &o);
    BeliefNode* getNodeChild();
    void getChildren(std::queue<BeliefNode*> &res);
    void write(std::ostream &os);
    void writeNGetChildren(std::ostream &os, std::queue<BeliefNode*> &res);

private:
    Observation vals;
    BeliefNode* child;
};

#endif /* OBSERVATIONEDGE_HPP */
