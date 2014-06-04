#ifndef SOLVER_SEARCHSTATUS_HPP_
#define SOLVER_SEARCHSTATUS_HPP_

namespace solver {

enum class SearchStatus : long {
    UNINITIALIZED,
    INITIAL,
    STAGE_COMPLETE,
    FINISHED,
    ERROR
};

} /* namespace solver */

#endif /* SOLVER_SEARCHSTATUS_HPP_ */
