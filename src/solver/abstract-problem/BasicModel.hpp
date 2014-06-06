#ifndef SOLVER_BASICMODEL_HPP_
#define SOLVER_BASICMODEL_HPP_

#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "global.hpp"                     // for RandomGenerator

#include "solver/abstract-problem/Model.hpp"

namespace solver {
class BasicModel : public Model {
public:
    BasicModel() = default;
    virtual ~BasicModel() = default;
    _NO_COPY_OR_MOVE(BasicModel);

    /* ---------------------- Basic customizations  ---------------------- */
    /** Approximates the value of a history entry based on the history and/or
     * an estimate using a single state. */
    virtual double getHeuristicValue(HistoricalData const *data,
            State const *state);

    /** Allows for a basic rollout strategy based on */
    virtual std::unique_ptr<Action> getRolloutAction(HistoricalData const *data,
            State const *state);

    /** Returns a vector of all of the */
    virtual std::vector<std::unique_ptr<DiscretizedPoint>> getAllActions();

    virtual std::vector<std::unique_ptr<Action>> getActionOrder();

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
