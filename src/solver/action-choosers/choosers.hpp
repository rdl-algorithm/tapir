#ifndef SOLVER_CHOOSERS_HPP_
#define SOLVER_CHOOSERS_HPP_

#include <memory>                       // for unique_ptr

#include "global.hpp"

#include "solver/abstract-problem/Action.hpp"

namespace solver {
class BeliefNode;
class Solver;

class ActionChoosingStrategy {
public:
    ActionChoosingStrategy() = default;
    virtual ~ActionChoosingStrategy() = default;
    /** Sets the action chooser for the given belief node. */
    virtual void setActionChooser(Solver *solver, BeliefNode *node) = 0;
};

class ActionChoosingFunction : public ActionChoosingStrategy {
public:
    ActionChoosingFunction(std::function<std::unique_ptr<Action>(BeliefNode const *)> function);

    virtual void setActionChooser(Solver *solver, BeliefNode *node);
private:
    std::function<std::unique_ptr<Action>(BeliefNode const *)> function_;
};


namespace choosers {
std::unique_ptr<Action> max_action(BeliefNode const *node);
std::unique_ptr<Action> robust_action(BeliefNode const *node);
std::unique_ptr<Action> ucb_action(BeliefNode const *node, double explorationCoefficient);
} /* namespace estimators */
} /* namespace solver */

#endif /* SOLVER_CHOOSERS_HPP_ */
