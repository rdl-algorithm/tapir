#ifndef SOLVER_MODELCHANGE_HPP_
#define SOLVER_MODELCHANGE_HPP_

#include "global.hpp"

namespace solver {
class ModelChange {
public:
    ModelChange() = default;
    virtual ~ModelChange() = default;
    _NO_COPY_OR_MOVE (ModelChange);
};
} /* namespace solver */

#endif /* SOLVER_MODELCHANGES_HPP_ */
