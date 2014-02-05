#ifndef SOLVER_OBSERVATIONEDGE_HPP_
#define SOLVER_OBSERVATIONEDGE_HPP_

#include <memory>                       // for unique_ptr

#include "Observation.hpp"              // for Observation

namespace solver {
class BeliefNode;

class ObservationEdge {
  public:
    friend class TextSerializer;

    /** Creates an observation edge without an observation or child!! */
    ObservationEdge();
    /** Constructs an observation edge with the given observation.
     */
    ObservationEdge(Observation const &obs);

    // Default destructor; copying and moving disallowed!
    ~ObservationEdge();
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

  private:
    /** The observation for this edge. */
    Observation observation_;
    /** The child belief node of this edge. */
    std::unique_ptr<BeliefNode> child_;
};
} /* namespace solver */

#endif /* SOLVER_OBSERVATIONEDGE_HPP_ */
