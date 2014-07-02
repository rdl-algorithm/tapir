/** @file State.hpp
 *
 * Defines the State interface; currently this is the same as Point, so this is just a straight
 * typedef.
 */
#ifndef SOLVER_STATE_HPP_
#define SOLVER_STATE_HPP_

#include "solver/abstract-problem/Point.hpp"

namespace solver {
    /** Currently State has no requirements over and above Point, so this is just a typedef. */
    typedef Point State;
} /* namespace solver */

#endif /* SOLVER_STATE_HPP_ */
