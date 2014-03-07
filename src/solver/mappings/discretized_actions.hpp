#ifndef SOLVER_DISCRETIZED_ACTIONS_HPP_
#define SOLVER_DISCRETIZED_ACTIONS_HPP_

#include <memory>
#include <vector>

#include "solver/serialization/Serializer.hpp"
#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/Model.hpp"

#include "ActionPool.hpp"
#include "ActionMapping.hpp"

#include "global.hpp"

namespace solver {
class ActionPool;
class ActionNode;
class EnumeratedPoint;

class ModelWithDiscretizedActions : virtual public solver::Model {
public:
    ModelWithDiscretizedActions() = default;
    virtual ~ModelWithDiscretizedActions() = default;
    _NO_COPY_OR_MOVE(ModelWithDiscretizedActions);

    virtual std::unique_ptr<ActionPool> createActionPool() override;
    virtual long getNumberOfBins() = 0;
    virtual std::unique_ptr<EnumeratedPoint> sampleAnAction(long code) = 0;
};

class DiscretizedActionPool: public solver::ActionPool {
  public:
    DiscretizedActionPool(ModelWithDiscretizedActions *model,
            long numberOfBins);
    virtual ~DiscretizedActionPool() = default;
    _NO_COPY_OR_MOVE(DiscretizedActionPool);

    virtual std::unique_ptr<ActionMapping> createActionMapping() override;
    virtual std::vector<long> generateActionOrder();
private:
  ModelWithDiscretizedActions *model_;
  long numberOfBins_;
};

class DiscretizedActionMap: public solver::ActionMapping {
  public:
    friend class DiscretizedActionTextSerializer;
    DiscretizedActionMap(ObservationPool *observationPool,
            ModelWithDiscretizedActions *model,
            long numberOfBins, std::vector<long> actionOrder);

    // Default destructor; copying and moving disallowed!
    virtual ~DiscretizedActionMap() = default;
    _NO_COPY_OR_MOVE(DiscretizedActionMap);

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
    virtual std::vector<ActionNode *> getChildren() const override;
  private:
    ObservationPool *observationPool_;
    ModelWithDiscretizedActions *model_;
    long numberOfBins_;

    std::vector<std::unique_ptr<ActionNode>> children_;
    long nChildren_;

    std::vector<long> actionOrder_;
    std::vector<long>::const_iterator nextActionIterator_;

    long bestActionCode_;
    double bestMeanQValue_;
};

class DiscretizedActionTextSerializer: virtual public solver::Serializer {
  public:
    DiscretizedActionTextSerializer() = default;
    virtual ~DiscretizedActionTextSerializer() = default;
    _NO_COPY_OR_MOVE(DiscretizedActionTextSerializer);

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

#endif /* SOLVER_DISCRETIZED_ACTIONS_HPP_ */
