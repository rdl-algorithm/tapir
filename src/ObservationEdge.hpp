#ifndef OBSERVATIONEDGE_HPP
#define OBSERVATIONEDGE_HPP

#include <queue>                        // for queue
#include "Observation.hpp"              // for Observation
class BeliefNode;

class ObservationEdge {
  public:
    friend class TextSerializer;

    ObservationEdge();
    ObservationEdge(Observation &o, BeliefNode *nxtBelNode);

    ~ObservationEdge() = default;
    ObservationEdge(ObservationEdge const &) = delete;
    ObservationEdge(ObservationEdge &&) = delete;
    ObservationEdge &operator=(ObservationEdge const &) = delete;
    ObservationEdge &operator=(ObservationEdge &&) = delete;

    bool isObs(Observation &o);
    BeliefNode *getNodeChild();
    void enqueueChildren(std::queue<BeliefNode *> &queue);

  private:
    Observation obs;
    BeliefNode *child;
};

#endif /* OBSERVATIONEDGE_HPP */
