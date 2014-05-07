#ifndef SOLVER_MODELCHANGE_HPP_
#define SOLVER_MODELCHANGE_HPP_

#include <map>
#include <memory>
#include <vector>

#include "global.hpp"

namespace solver {
class ModelChange {
public:
    ModelChange() = default;
    virtual ~ModelChange() = default;
    _NO_COPY_OR_MOVE (ModelChange);
};

typedef std::map<long, std::vector<std::unique_ptr<ModelChange>>> ChangeSequence;
} /* namespace solver */

#endif /* SOLVER_MODELCHANGES_HPP_ */

