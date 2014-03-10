#ifndef SOLVER_SEARCHSTATUS_HPP_
#define SOLVER_SEARCHSTATUS_HPP_

namespace solver {

enum class SearchStatus : long {
    INITIAL = 0,
    INSIDE_TREE = 1,
    REACHED_ROLLOUT_NODE = 2,
    ROLLING_OUT = 3,
    ROLLOUT_COMPLETE = 4,
    TERMINATED = 5,
    HIT_DEPTH_LIMIT = 6
};

} /* namespace solver */

#endif /* SOLVER_SEARCHSTATUS_HPP_ */
