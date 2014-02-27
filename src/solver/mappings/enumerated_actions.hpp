#ifndef SOLVER_ENUMERATED_ACTIONS_HPP_
#define SOLVER_ENUMERATED_ACTIONS_HPP_

#include <memory>
#include <vector>

#include "solver/serialization/Serializer.hpp"
#include "solver/geometry/Action.hpp"
#include "solver/Model.hpp"

#include "ActionPool.hpp"
#include "ActionMapping.hpp"

#include "global.hpp"

namespace solver {
class ActionPool;
class ActionNode;
class EnumeratedPoint;

class ModelWithEnumeratedActions : virtual public solver::Model {
public:
    ModelWithEnumeratedActions() = default;
    virtual ~ModelWithEnumeratedActions() = default;
    _NO_COPY_OR_MOVE(ModelWithEnumeratedActions);

    virtual std::unique_ptr<ActionPool> createActionPool() override;
    virtual std::vector<std::unique_ptr<EnumeratedPoint>>
    getAllActionsInOrder() = 0;
};

class EnumeratedActionPool: public solver::ActionPool {
    friend class TextSerializer;
  public:
    EnumeratedActionPool(RandomGenerator *randGen,
            std::vector<std::unique_ptr<EnumeratedPoint>> actions);
    virtual ~EnumeratedActionPool() = default;
    _NO_COPY_OR_MOVE(EnumeratedActionPool);

    virtual std::unique_ptr<ActionMapping> createActionMapping() override;
    virtual std::vector<long> generateActionOrder();
private:
  RandomGenerator *randGen_;
  std::vector<std::unique_ptr<EnumeratedPoint>> allActions_;
};

class EnumeratedActionMap: public solver::ActionMapping {
  public:
    friend class TextSerializer;
    friend class EnumeratedActionTextSerializer;
    EnumeratedActionMap(ObservationPool *observationPool,
            std::vector<std::unique_ptr<EnumeratedPoint>> const &actions,
            std::vector<long> actionOrder);

    // Default destructor; copying and moving disallowed!
    virtual ~EnumeratedActionMap() = default;
    _NO_COPY_OR_MOVE(EnumeratedActionMap);

    virtual ActionNode *getActionNode(Action const &action) const override;
    virtual ActionNode *createActionNode(Action const &action) override;

    virtual long getNChildren() const override;
    virtual long size() const;

    virtual bool hasActionToTry() const override;
    virtual std::unique_ptr<Action> getNextActionToTry() override;

    virtual std::unique_ptr<Action> getSearchAction(
            double exploreCofficient) override;

    virtual void updateBestValue() override;
    virtual std::unique_ptr<Action> getBestAction() const override;
    virtual double getBestMeanQValue() const override;
  private:
    std::vector<std::unique_ptr<EnumeratedPoint>> const &allActions_;
    ObservationPool *observationPool_;

    std::vector<std::unique_ptr<ActionNode>> children_;
    long nChildren_;

    std::vector<long> actionOrder_;
    std::vector<long>::const_iterator nextActionIterator_;

    Action const *bestAction_;
    double bestMeanQValue_;
};

class EnumeratedActionTextSerializer: virtual public solver::Serializer {
  public:
    EnumeratedActionTextSerializer() = default;
    virtual ~EnumeratedActionTextSerializer() = default;
    _NO_COPY_OR_MOVE(EnumeratedActionTextSerializer);

    virtual void saveActionPool(
            ActionPool const &actionPool, std::ostream &os) override;
    virtual std::unique_ptr<ActionPool> loadActionPool(
            std::istream &is) override;
    virtual void saveActionMapping(ActionMapping const &map,
            std::ostream &os) override;
    virtual std::unique_ptr<ActionMapping> loadActionMapping(
            std::istream &is) override;
};
} /* namespace solver */

#endif /* SOLVER_ENUMERATED_ACTIONS_HPP_ */
