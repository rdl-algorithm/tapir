#ifndef SOLVER_SERIALIZER_HPP_
#define SOLVER_SERIALIZER_HPP_

#include <istream>                      // for istream, ostream
#include <memory>                       // for unique_ptr

#include "solver/abstract-problem/Observation.hpp"              // for Observation
#include "solver/abstract-problem/State.hpp"
#include "solver/Solver.hpp"                   // for Solver

#include "global.hpp"

namespace solver {
class ActionMapping;
class ActionNode;
class BeliefNode;
class BeliefTree;
class Histories;
class HistoryEntry;
class HistorySequence;
class ObservationMapping;
class StateInfo;
class StatePool;

class Serializer {
public:
    /** Constructs a serializer for the given solver. */
    Serializer() :
            solver_(nullptr),
            model_(nullptr) {
    }

    Serializer(Solver *solver) :
            solver_(solver),
            model_(solver_->model_.get()) {
    }

    void setSolver(Solver *solver) {
        solver_ = solver;
    }

    Solver *getSolver() {
        return solver_;
    }

    /** Default destructor. */
    virtual ~Serializer() = default;

    /* Copying and moving is disallowed. */
    _NO_COPY_OR_MOVE(Serializer);

    /* --------------- Saving the entirnode.e solver. ----------------- */

    /** Saves the sate of the solver. */
    virtual void save(std::ostream &os) {
        save(*(solver_->statePool_), os);
        save(*(solver_->histories_), os);
        saveActionPool(*(solver_->actionPool_), os);
        saveObservationPool(*(solver_->observationPool_), os);
        save(*(solver_->policy_), os);
    }
    /** Loads the state of the solver. */
    virtual void load(std::istream &is) {
        load(*(solver_->statePool_), is);
        load(*(solver_->histories_), is);
        solver_->actionPool_ = loadActionPool(is);
        solver_->observationPool_ = loadObservationPool(is);
        solver_->initialize();
        load(*(solver_->policy_), is);
    }

    /* --------------- Saving states & observations ----------------- */

    // NOTE: null values need to be handled for saving of observations
    // and of actions

    /** Saves a State. */
    virtual void saveState(State const *state, std::ostream &os) = 0;
    /** Loads a State. */
    virtual std::unique_ptr<State> loadState(std::istream &is) = 0;

    /** Saves an Action. */
    virtual void saveAction(Action const *action, std::ostream &os) = 0;
    /** Loads an Action. */
    virtual std::unique_ptr<Action> loadAction(std::istream &is) = 0;

    /** Saves TransitionParameters. */
    virtual void saveTransitionParameters(TransitionParameters const *tp,
            std::ostream &os) = 0;
    /** Loads TransitionParameters. */
    virtual std::unique_ptr<TransitionParameters> loadTransitionParameters(
            std::istream &is) = 0;

    /** Saves an Observation. */
    virtual void saveObservation(Observation const *obs, std::ostream &os) = 0;
    /** Loads an Observation. */
    virtual std::unique_ptr<Observation> loadObservation(std::istream &is) = 0;


    /** Saves the pool handling all the actions. */
    virtual void saveActionPool(ActionPool const &actionPool,
            std::ostream &os) = 0;
    /** Loads the pool handling all the actions. */
    virtual  std::unique_ptr<ActionPool> loadActionPool(std::istream &is) = 0;
    /** Saves a mapping of observations to belief nodes. */
    virtual void saveActionMapping(ActionMapping const &map,
            std::ostream &os) = 0;
    /** Loads a mapping of observations to belief nodes. */
    virtual std::unique_ptr<ActionMapping> loadActionMapping(
            std::istream &is) = 0;

    /** Saves the pool handling all the actions. */
    virtual void saveObservationPool(
            ObservationPool const &observationPool, std::ostream &os) = 0;
    /** Loads the pool handling all the actions. */
    virtual std::unique_ptr<ObservationPool> loadObservationPool(
            std::istream &is) = 0;
    /** Saves a mapping of observations to belief nodes. */
    virtual void saveObservationMapping(ObservationMapping const &map,
            std::ostream &os) = 0;
    /** Loads a mapping of observations to belief nodes. */
    virtual std::unique_ptr<ObservationMapping> loadObservationMapping(
            std::istream &is) = 0;

    /* --------------- Saving the state pool ----------------- */

    /** Saves a StateInfo. */
    virtual void save(StateInfo const &wrapper, std::ostream &os) = 0;
    /** Loads a StateInfo. */
    virtual void load(StateInfo &wrapper, std::istream &is) = 0;
    /** Saves a StatePool. */
    virtual void save(StatePool const &pool, std::ostream &os) = 0;
    /** Loads a StatePool. */
    virtual void load(StatePool &pool, std::istream &is) = 0;


    /* --------------- Saving the history sequences ----------------- */

    /** Saves a HistoryEntry. */
    virtual void save(HistoryEntry const &entry, std::ostream &os) = 0;
    /** Loads a HistoryEntry. */
    virtual void load(HistoryEntry &entry, std::istream &is) = 0;
    /** Saves a HistorySequence. */
    virtual void save(HistorySequence const &seq, std::ostream &os) = 0;
    /** Loads a HistorySequence. */
    virtual void load(HistorySequence &seq, std::istream &is) = 0;
    /** Saves a Histories. */
    virtual void save(Histories const &histories, std::ostream &os) = 0;
    /** Loads a Histories. */
    virtual void load(Histories &histories, std::istream &is) = 0;

    /* --------------- Saving the policy tree ----------------- */

    /** Saves an ActionNode. */
    virtual void save(ActionNode const &node, std::ostream &os) = 0;
    /** Loads an ActionNode. */
    virtual void load(ActionNode &node, std::istream &is) = 0;
    /** Saves a BeliefNode. */
    virtual void save(BeliefNode const &node, std::ostream &os) = 0;
    /** Loads a BeliefNode. */
    virtual void load(BeliefNode &node, std::istream &is) = 0;
    /** Saves a BeliefTree. */
    virtual void save(BeliefTree const &tree, std::ostream &os) = 0;
    /** Loads a BeliefTree. */
    virtual void load(BeliefTree &tree, std::istream &is) = 0;
  protected:
    Solver *solver_;
    Model *model_;
};
} /* namespace solver */

#endif /* SOLVER_SERIALIZER_HPP_ */
