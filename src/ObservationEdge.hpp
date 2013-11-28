#ifndef OBSERVATIONEDGE_HPP
#define OBSERVATIONEDGE_HPP

#include <queue>

#include "Observation.hpp"
class BeliefNode;

class ObservationEdge {
public:
    friend class TextSerializer;

    ObservationEdge();
    ObservationEdge(Observation &o, BeliefNode* nxtBelNode);

    ~ObservationEdge() = default;
    ObservationEdge(const ObservationEdge&) = delete;
    ObservationEdge(ObservationEdge&) = delete;
    ObservationEdge &operator=(const ObservationEdge&) = delete;
    ObservationEdge &operator=(ObservationEdge&) = delete;

    bool isObs(Observation &o);
    BeliefNode* getNodeChild();
    void enqueueChildren(std::queue<BeliefNode*> &queue);

private:
    Observation obs;
    BeliefNode* child;
};

#endif /* OBSERVATIONEDGE_HPP */
