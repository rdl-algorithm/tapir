#ifndef SOLVER_ENUMERATED_ACTIONS_HPP_
#define SOLVER_ENUMERATED_ACTIONS_HPP_

#include <memory>
#include <vector>

#include "solver/serialization/Serializer.hpp"
#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/Model.hpp"

#include "ActionPool.hpp"
#include "ActionMapping.hpp"

#include "global.hpp"
#include "RandomAccessSet.hpp"

namespace solver {
class ActionPool;
class ActionNode;
class EnumeratedActionMapEntry;
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
  public:
    EnumeratedActionPool(RandomGenerator *randGen,
            double ucbExplorationCoefficient,
            std::vector<std::unique_ptr<EnumeratedPoint>> actions);
    virtual ~EnumeratedActionPool() = default;
    _NO_COPY_OR_MOVE(EnumeratedActionPool);

    virtual std::unique_ptr<ActionMapping> createActionMapping() override;
private:
  RandomGenerator *randGen_;
  double ucbExplorationCoefficient_;
  std::vector<std::unique_ptr<EnumeratedPoint>> allActions_;
};

class EnumeratedActionMap: public solver::ActionMapping {
  public:
    friend class EnumeratedActionTextSerializer;
    EnumeratedActionMap(ObservationPool *observationPool,
            double explorationCoefficient,
            std::vector<std::unique_ptr<EnumeratedPoint>> const &actions);

    // Default destructor; copying and moving disallowed!
    virtual ~EnumeratedActionMap() = default;
    _NO_COPY_OR_MOVE(EnumeratedActionMap);

    virtual ActionNode *getActionNode(Action const &action) const override;
    virtual ActionNode *createActionNode(Action const &action) override;

    virtual long getNChildren() const override;
    virtual std::vector<ActionMappingEntry const *> getChildEntries() const override;

    virtual bool hasRolloutActions() const override;
    virtual std::vector<std::unique_ptr<Action>> getRolloutActions() const override;

    virtual void updateBestValue() override;
    virtual std::unique_ptr<Action> getBestAction() const override;
    virtual double getBestMeanQValue() const override;
  private:
    ObservationPool *observationPool_;
    double ucbExplorationCoefficient_;
    std::vector<std::unique_ptr<EnumeratedPoint>> const &allActions_;

    std::vector<std::unique_ptr<EnumeratedActionMapEntry>> entries_;
    long nChildren_;

    abt::RandomAccessSet<long> actionsToTry_;

    Action const *bestAction_;
    double bestMeanQValue_;
};

class EnumeratedActionMapEntry : virtual public solver::ActionMappingEntry {
  public:
    EnumeratedActionMapEntry(Action const &action,
            std::unique_ptr<ActionNode> childNode);

    virtual std::unique_ptr<Action> getAction() const override;
    virtual ActionNode *getActionNode() const override;
  private:
    std::unique_ptr<Action> action_;
    std::unique_ptr<ActionNode> childNode_;
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
