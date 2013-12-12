#ifndef SOLVER_SERIALIZER_HPP_
#define SOLVER_SERIALIZER_HPP_

#include <istream>                      // for istream, ostream
#include <memory>                       // for unique_ptr

#include "Observation.hpp"              // for Observation
#include "Solver.hpp"                   // for Solver

namespace solver {
class ActionNode;
class BeliefNode;
class BeliefTree;
class Histories;
class HistoryEntry;
class HistorySequence;
class ObservationEdge;
class State;
class StateInfo;
class StatePool;

class Serializer {
  public:
    /** Constructs a serializer for the given solver. */
    Serializer(Solver *solver) :
        solver_(solver) {
    }
    /** Default destructor. */
    virtual ~Serializer() = default;

    /* Copying and moving is disallowed. */
    Serializer(Serializer const &) = delete;
    Serializer(Serializer &&) = delete;
    Serializer &operator=(Serializer const &) = delete;
    Serializer &operator=(Serializer &&) = delete;

    /** Saves the sate of the solver. */
    virtual void save(std::ostream &os) {
        save(*(solver_->allStates_), os);
        save(*(solver_->allHistories_), os);
        save(*(solver_->policy_), os);
    }
    /** Loads the state of the solver. */
    virtual void load(std::istream &is) {
        load(*(solver_->allStates_), is);
        load(*(solver_->allHistories_), is);
        load(*(solver_->policy_), is);
    }

    /** Saves a State. */
    virtual void saveState(State &state, std::ostream &os) = 0;
    /** Loads a State. */
    virtual std::unique_ptr<State> loadState(std::istream &is) = 0;

    /** Saves a StateInfo. */
    virtual void save(StateInfo &wrapper, std::ostream &os) = 0;
    /** Loads a StateInfo. */
    virtual void load(StateInfo &wrapper, std::istream &is) = 0;
    /** Saves a StatePool. */
    virtual void save(StatePool &pool, std::ostream &os) = 0;
    /** Loads a StatePool. */
    virtual void load(StatePool &pool, std::istream &is) = 0;

    virtual void save(Observation &obs, std::ostream &os) = 0;
    virtual void load(Observation &obs, std::istream &is) = 0;
    virtual void save(HistoryEntry &entry, std::ostream &os) = 0;
    virtual void load(HistoryEntry &entry, std::istream &is) = 0;
    virtual void save(HistorySequence &seq, std::ostream &os) = 0;
    virtual void load(HistorySequence &seq, std::istream &is) = 0;
    virtual void save(Histories &histories, std::ostream &os) = 0;
    virtual void load(Histories &histories, std::istream &is) = 0;

    virtual void save(ObservationEdge &edge, std::ostream &os) = 0;
    virtual void load(ObservationEdge &edge, std::istream &is) = 0;
    virtual void save(ActionNode &node, std::ostream &os) = 0;
    virtual void load(ActionNode &node, std::istream &is) = 0;
    virtual void save(BeliefNode &node, std::ostream &os) = 0;
    virtual void load(BeliefNode &node, std::istream &is) = 0;
    virtual void save(BeliefTree &tree, std::ostream &os) = 0;
    virtual void load(BeliefTree &tree, std::istream &is) = 0;
  protected:
    Solver *solver_;
};
} /* namespace solver */

#endif /* SOLVER_SERIALIZER_HPP_ */
