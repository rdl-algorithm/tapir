#ifndef SOLVER_ENUMERATED_ACTIONS_HPP_
#define SOLVER_ENUMERATED_ACTIONS_HPP_

#include <memory>
#include <vector>

#include "discretized_actions.hpp"

#include "solver/serialization/Serializer.hpp"
#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/DiscretizedPoint.hpp"
#include "solver/abstract-problem/Model.hpp"

#include "ActionPool.hpp"
#include "ActionMapping.hpp"

#include "global.hpp"
#include "RandomAccessSet.hpp"

namespace solver {
class ActionPool;

class ModelWithEnumeratedActions : virtual public solver::ModelWithDiscretizedActions {
  public:
    ModelWithEnumeratedActions(std::vector<std::unique_ptr<DiscretizedPoint>> allActions);
    virtual ~ModelWithEnumeratedActions() = default;
    _NO_COPY_OR_MOVE(ModelWithEnumeratedActions);

    virtual long getNumberOfBins() override;
    virtual std::unique_ptr<Action> sampleAnAction(long binNumber) override;

    virtual void setAllActions(std::vector<std::unique_ptr<DiscretizedPoint>> allActions);
  private:
    std::vector<std::unique_ptr<DiscretizedPoint>> allActions_;
};
} /* namespace solver */

#endif /* SOLVER_ENUMERATED_ACTIONS_HPP_ */
