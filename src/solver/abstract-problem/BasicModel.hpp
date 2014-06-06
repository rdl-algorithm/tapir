#ifndef SOLVER_BASICMODEL_HPP_
#define SOLVER_BASICMODEL_HPP_

#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "global.hpp"                     // for RandomGenerator

#include "solver/abstract-problem/Model.hpp"

namespace solver {
class BasicModel: public virtual Model {
public:
    BasicModel() = default;
    virtual ~BasicModel() = default;
    _NO_COPY_OR_MOVE(BasicModel);

    /* ---------------------- Basic customizations  ---------------------- */
    /** Approximates the value of a history entry based on the history and/or
     * an estimate using a single state. */
    virtual double getHeuristicValue(HistoricalData const *data, State const *state);

    /** Allows for a basic rollout strategy based on the history and/or the specific state. */
    virtual std::unique_ptr<Action> getRolloutAction(HistoricalData const *data,
            State const *state);


    /* ---- Basic implementations for action/observation mappings. ---- */
    /** Returns a vector of all possible actions. */
    virtual std::vector<std::unique_ptr<DiscretizedPoint>> getAllActions() = 0;

    /** Returns the actions to try for the given history, in the order in which they should
     * be tried. */
    virtual std::vector<std::unique_ptr<Action>> getActionOrder(HistoricalData const *data);

    /* ------- Customization of more complex solver functionality  --------- */
    virtual std::unique_ptr<ActionPool> createActionPool(Solver *solver);
    /** Creates an ObservationPool, which manages observations and creates
     * ObservationMappings.
     */
    virtual std::unique_ptr<ObservationPool> createObservationPool(Solver *solver);

    virtual std::unique_ptr<SearchStrategy> createSearchStrategy(Solver *solver);
};
} /* namespace solver */

#endif /* SOLVER_MODEL_HPP_ */
