#ifndef SOLVER_CHOOSERS_HPP_
#define SOLVER_CHOOSERS_HPP_

#include <memory>                       // for unique_ptr

#include "global.hpp"

#include "solver/abstract-problem/Action.hpp"

namespace solver {
class BeliefNode;

namespace choosers {
std::unique_ptr<Action> max_action(BeliefNode const *node);
std::unique_ptr<Action> robust_action(BeliefNode const *node);
std::unique_ptr<Action> ucb_action(BeliefNode const *node, double explorationCoefficient);
} /* namespace estimators */
} /* namespace solver */

#endif /* SOLVER_CHOOSERS_HPP_ */
