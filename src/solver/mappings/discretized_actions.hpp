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

    /* -------------- Creation and retrieval of nodes. ---------------- */
    virtual ActionNode *getActionNode(Action const &action) const override;
    virtual ActionNode *createActionNode(Action const &action) override;

    /* -------------- Retrieval of mapping entries. ---------------- */
    virtual long getNChildren() const override;
    virtual std::vector<ActionMappingEntry const *> getChildEntries() const override;
    virtual ActionMappingEntry const *getEntry(Action const &action) const override;

    /* -------------- Retrieval of general statistics. ---------------- */
    virtual long getTotalVisitCount() const override;
    virtual std::unique_ptr<Action> getBestAction() const override;
    virtual double getBestMeanQValue() const override;

    /* ------------ Methods for retrieving unvisited actions -------------- */
    virtual bool hasUnvisitedActions() const override;
    virtual std::vector<std::unique_ptr<Action>> getUnvisitedActions() const override;
    virtual std::unique_ptr<Action> getRandomUnvisitedAction() const override;

    virtual void deleteUnvisitedAction(long binNumber);
    virtual void addUnvisitedAction(long binNumber);

    /* ------------ Easy getters for entry values. -------------- */
    virtual long getVisitCount(Action const &action) const override;
    virtual double getTotalQValue(Action const &action) const override;
    virtual double getMeanQValue(Action const &action) const override;

    /* --------------- Methods for updating the values ----------------- */
    virtual void updateVisitCount(Action const &action, long deltaNVisits) override;
    virtual void updateTotalQValue(Action const &action, double deltaQ) override;
    virtual void updateQValue() override;

  private:
    ObservationPool *observationPool_;
    ModelWithDiscretizedActions *model_;
    long numberOfBins_;

    std::vector<std::unique_ptr<DiscretizedActionMapEntry>> entries_;
    long nChildren_;

    abt::RandomAccessSet<long> binsToTry_;

    long bestBinNumber_;
    double bestMeanQValue_;

    long totalVisitCount_;
};

class DiscretizedActionMapEntry : virtual public solver::ActionMappingEntry {
    friend class DiscretizedActionMap;
    friend class DiscretizedActionTextSerializer;
  public:
    DiscretizedActionMapEntry(long binNumber,
            DiscretizedActionMap *map,
            std::unique_ptr<ActionNode> childNode);
    virtual ~DiscretizedActionMapEntry() = default;
    _NO_COPY_OR_MOVE(DiscretizedActionMapEntry);

    virtual std::unique_ptr<Action> getAction() const override;
    virtual ActionNode *getActionNode() const override;
    virtual long getVisitCount() const override;
    virtual double getTotalQValue() const override;
    virtual double getMeanQValue() const override;

    virtual long getBinNumber() const;
  private:
    long binNumber_;
    DiscretizedActionMap *map_;
    std::unique_ptr<ActionNode> childNode_;
    long visitCount_;
    double totalQValue_;
    double meanQValue_;
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
