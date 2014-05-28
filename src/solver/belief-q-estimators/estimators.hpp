#ifndef SOLVER_ESTIMATORS_HPP_
#define SOLVER_ESTIMATORS_HPP_

#include <memory>                       // for unique_ptr

#include "global.hpp"

namespace solver {
class BeliefNode;

namespace estimators {
double average_q_value(BeliefNode const *node);
double max_q_value(BeliefNode const *node);
double robust_q_value(BeliefNode const *node);
} /* namespace estimators */
} /* namespace solver */

#endif /* SOLVER_ESTIMATORS_HPP_ */
