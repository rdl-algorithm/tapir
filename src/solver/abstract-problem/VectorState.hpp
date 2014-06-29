/** file: VectorState.hpp
 *
 * Defines the VectorState interface; currently this is the same as Vector, so this is just a
 * straight typedef.
 */
#ifndef SOLVER_VECTORSTATE_HPP_
#define SOLVER_VECTORSTATE_HPP_

#include "solver/abstract-problem/Vector.hpp"

namespace solver {
    typedef Vector VectorState;
}

#endif /* SOLVER_VECTORSTATE_HPP_ */
