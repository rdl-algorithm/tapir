#ifndef OBSERVATIONEDGE_HPP
#define OBSERVATIONEDGE_HPP

#include <memory>                       // for unique_ptr
#include <queue>                        // for queue

// #include "BeliefNode.hpp"               // for BeliefNode
#include "Observation.hpp"              // for Observation
class BeliefNode;

class ObservationEdge {
  public:
    friend class TextSerializer;

    /** Constructs an observation edge with the given observation.
     */
    ObservationEdge(Observation const &obs);

    /** Default destructor. */
    ~ObservationEdge() = default;

    /* Copying and moving is disallowed. */
    ObservationEdge(ObservationEdge const &) = delete;
    ObservationEdge(ObservationEdge &&) = delete;
    ObservationEdge &operator=(ObservationEdge const &) = delete;
    ObservationEdge &operator=(ObservationEdge &&) = delete;

    /** Checks whether this edge's observation is considered close enough
     * to be equivalent to the given one.
     */
    bool obsEquals(Observation const &otherObs);

    /** Returns the belief child of this node */
    BeliefNode *getBeliefChild();
    /** Enqueues the child of this node on the given queue. */
    void enqueueChildren(std::queue<BeliefNode *> &queue);

  private:
    /** Creates an observation edge without an observation or child! */
    ObservationEdge();
    /** The observation for this edge. */
    Observation obs;
    /** The child belief node of this edge. */
    std::unique_ptr<BeliefNode> child;
};

#endif /* OBSERVATIONEDGE_HPP */
