#ifndef SOLVER_DISCRETIZED_ACTIONS_HPP_
#define SOLVER_DISCRETIZED_ACTIONS_HPP_

#include <memory>
#include <vector>

#include "solver/serialization/Serializer.hpp"
#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/Model.hpp"

#include "ActionPool.hpp"
#include "ActionMapping.hpp"

#include "RandomAccessSet.hpp"
#include "global.hpp"

namespace solver {
class ActionPool;
class ActionNode;
class DiscretizedPoint;
class DiscretizedActionMapEntry;

class ModelWithDiscretizedActions : virtual public solver::Model {
public:
    ModelWithDiscretizedActions() = default;
    virtual ~ModelWithDiscretizedActions() = default;
    _NO_COPY_OR_MOVE(ModelWithDiscretizedActions);

    virtual std::unique_ptr<ActionPool> createActionPool() override;
    virtual long getNumberOfBins() = 0;
    virtual std::unique_ptr<Action> sampleAnAction(long binNumber) = 0;
};

class DiscretizedActionPool: public solver::ActionPool {
  public:
    DiscretizedActionPool(ModelWithDiscretizedActions *model,
            long numberOfBins);
    virtual ~DiscretizedActionPool() = default;
    _NO_COPY_OR_MOVE(DiscretizedActionPool);

    virtual std::unique_ptr<ActionMapping> createActionMapping() override;
private:
  ModelWithDiscretizedActions *model_;
  long numberOfBins_;
};

class DiscretizedActionMap: public solver::ActionMapping {
  public:
    friend class DiscretizedActionTextSerializer;
    friend class DiscretizedActionMapEntry;

    DiscretizedActionMap(ObservationPool *observationPool,
            ModelWithDiscretizedActions *model,
            long numberOfBins);

    // Default destructor; copying and moving disallowed!
    virtual ~DiscretizedActionMap() = default;
    _NO_COPY_OR_MOVE(DiscretizedActionMap);

    virtual ActionNode *getActionNode(Action const &action) const override;
    virtual ActionNode *createActionNode(Action const &action) override;

    virtual long getNChildren() const override;
    virtual std::vector<ActionMappingEntry const *> getChildEntries() const override;

    virtual bool hasRolloutActions() const override;
    virtual std::vector<std::unique_ptr<Action>> getRolloutActions() const override;
    virtual std::unique_ptr<Action> getRandomRolloutAction() const override;

    virtual void update() override;
    virtual std::unique_ptr<Action> getBestAction() const override;
    virtual double getBestMeanQValue() const override;
  private:
    ObservationPool *observationPool_;
    ModelWithDiscretizedActions *model_;
    long numberOfBins_;

    std::vector<std::unique_ptr<DiscretizedActionMapEntry>> entries_;
    long nChildren_;

    abt::RandomAccessSet<long> binsToTry_;

    long bestBinNumber;
    double bestMeanQValue_;
};

class DiscretizedActionMapEntry : virtual public solver::ActionMappingEntry {
  public:
    DiscretizedActionMapEntry(long binNumber,
            DiscretizedActionMap *map,
            std::unique_ptr<ActionNode> childNode);
    virtual ~DiscretizedActionMapEntry() = default;
    _NO_COPY_OR_MOVE(DiscretizedActionMapEntry);

    virtual std::unique_ptr<Action> getAction() const override;
    virtual ActionNode *getActionNode() const override;

    virtual long getBinNumber() const;
  private:
    long binNumber_;
    DiscretizedActionMap *map_;
    std::unique_ptr<ActionNode> childNode_;
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
