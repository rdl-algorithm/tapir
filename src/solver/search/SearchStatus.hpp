#ifndef SOLVER_SEARCHSTATUS_HPP_
#define SOLVER_SEARCHSTATUS_HPP_

namespace solver {

enum class SearchStatus : long {
    UNINITIALIZED,
    INITIAL,
    INSIDE_TREE,
    REACHED_LEAF,
    ROLLING_OUT,
    ROLLOUT_COMPLETE,
    HIT_TERMINAL_STATE,
    HIT_DEPTH_LIMIT,
    ERROR
};

} /* namespace solver */

#endif /* SOLVER_SEARCHSTATUS_HPP_ */
