/** @file choosers.hpp
 *
 * Defines some useful basic functions for selecting an action from a belief node.
 */
#ifndef SOLVER_CHOOSERS_HPP_
#define SOLVER_CHOOSERS_HPP_

#include <memory>                       // for unique_ptr

#include "global.hpp"

#include "solver/abstract-problem/Action.hpp"

namespace solver {
class BeliefNode;
class Solver;

namespace choosers {
/** Returns the action with the maximum estimate Q-value. */
std::unique_ptr<Action> max_action(BeliefNode const *node);
/** Returns the action with the highest visit count (ties are broken by max. value) */
std::unique_ptr<Action> robust_action(BeliefNode const *node);
/** Returns the action with the highest UCB value, using the given exploration coefficient. */
std::unique_ptr<Action> ucb_action(BeliefNode const *node, double explorationCoefficient);
} /* namespace choosers */
} /* namespace solver */

#endif /* SOLVER_CHOOSERS_HPP_ */
