#ifndef SOLVER_CHANGETYPE_HPP_
#define SOLVER_CHANGETYPE_HPP_

namespace solver {
enum class ChangeType {
    UNDEFINED = 0,
    ADDSTATE = 1,
    REWARD = 2,
    TRANSITION = 3,
    DELSTATE = 4,
    ADDOBSERVATION = 5,
    ADDOBSTACLE = 6
};
} /* namespace solver */

#endif /* SOLVER_CHANGETYPE_HPP_ */
