/** file: Observation.hpp
 *
 * Defines the Observation interface; currently this is the same as Point, so this is just a
 * straight typedef.
 */
#ifndef SOLVER_OBSERVATION_HPP_
#define SOLVER_OBSERVATION_HPP_

#include "solver/abstract-problem/Point.hpp"

namespace solver {
    /** Currently Observation has no requirements over and above Point,
     * so this is just a typedef.
     */
    typedef Point Observation;
} /* namespace solver */

#endif /* SOLVER_OBSERVATION_HPP_ */
