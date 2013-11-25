#ifndef OBSERVATIONEDGE_H
#define OBSERVATIONEDGE_H

#include <ostream>
#include <queue>

#include "Model.h"

class BeliefNode;

class ObservationEdge {
public:
    ObservationEdge();
    ObservationEdge(ObsVals &o, BeliefNode* nxtBelNode);

    ~ObservationEdge() = default;
    ObservationEdge(const ObservationEdge&) = delete;
    ObservationEdge(ObservationEdge&) = delete;
    ObservationEdge &operator=(const ObservationEdge&) = delete;
    ObservationEdge &operator=(ObservationEdge&) = delete;

    bool isObs(ObsVals &o);
    BeliefNode* getNodeChild();
    void getChildren(std::queue<BeliefNode*> &res);
    void write(std::ostream &os);
    void writeNGetChildren(std::ostream &os, std::queue<BeliefNode*> &res);

private:
    ObsVals vals;
    BeliefNode* child;
};
#endif
