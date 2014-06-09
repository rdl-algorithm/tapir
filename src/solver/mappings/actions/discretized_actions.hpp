#ifndef SOLVER_DISCRETIZED_ACTIONS_HPP_
#define SOLVER_DISCRETIZED_ACTIONS_HPP_

#include <memory>
#include <vector>

#include "solver/serialization/Serializer.hpp"
#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/Model.hpp"

#include "solver/ActionNode.hpp"

#include "ActionPool.hpp"
#include "ActionMapping.hpp"

#include "RandomAccessSet.hpp"
#include "global.hpp"

namespace solver {
class ActionPool;
class DiscretizedPoint;
class DiscretizedActionMapEntry;

class DiscretizedActionPool: public solver::ActionPool {
  public:
    DiscretizedActionPool() = default;
    virtual ~DiscretizedActionPool() = default;
    _NO_COPY_OR_MOVE(DiscretizedActionPool);

    virtual long getNumberOfBins() = 0;
    virtual std::unique_ptr<Action> sampleAnAction(long binNumber) = 0;
    virtual std::vector<long> createBinSequence(HistoricalData const *data) = 0;

    virtual std::unique_ptr<ActionMapping> createActionMapping(BeliefNode *node) override;
};

class DiscretizedActionMap: public solver::ActionMapping {
  public:
    friend class DiscretizedActionTextSerializer;
    friend class DiscretizedActionMapEntry;

    DiscretizedActionMap(DiscretizedActionPool *pool, std::vector<long> binSequence);

    /* -------------- Association with a belief node ---------------- */
    virtual void setOwner(BeliefNode *owner) override;
    virtual BeliefNode *getOwner() const override;

    // Default destructor; copying and moving disallowed!
    virtual ~DiscretizedActionMap();
    _NO_COPY_OR_MOVE(DiscretizedActionMap);

    /* -------------- Creation and retrieval of nodes. ---------------- */
    virtual ActionNode *getActionNode(Action const &action) const override;
    virtual ActionNode *createActionNode(Action const &action) override;
    virtual long getNChildren() const override;

    /* -------------- Retrieval of mapping entries. ---------------- */
    virtual long getNumberOfVisitedEntries() const override;
    virtual std::vector<ActionMappingEntry const *> getVisitedEntries() const override;
    virtual ActionMappingEntry *getEntry(Action const &action) override;
    virtual ActionMappingEntry const *getEntry(Action const &action) const override;

    /* ----------------- Methods for unvisited actions ------------------- */
    /** Returns the next action to be tried for this node, or nullptr if there are no more. */
    virtual std::unique_ptr<Action> getNextActionToTry() override;

    /* -------------- Retrieval of general statistics. ---------------- */
    virtual long getTotalVisitCount() const override;

  protected:
    BeliefNode *owningBeliefNode_;
    DiscretizedActionPool *pool_;
    long numberOfBins_;

    std::vector<DiscretizedActionMapEntry> entries_;
    long nChildren_;
    long numberOfVisitedEntries_;

    std::vector<long> binSequence_;
    std::vector<long>::const_iterator binIterator_;

    long totalVisitCount_;
};

class DiscretizedActionMapEntry : virtual public solver::ActionMappingEntry {
    friend class DiscretizedActionMap;
    friend class DiscretizedActionTextSerializer;
  public:
    virtual ActionMapping *getMapping() const override;
    virtual std::unique_ptr<Action> getAction() const override;
    virtual ActionNode *getActionNode() const override;
    virtual long getVisitCount() const override;
    virtual double getTotalQValue() const override;
    virtual double getMeanQValue() const override;

    virtual long getBinNumber() const;

    virtual bool updateValue(long deltaNVisits, double deltaTotalQ) override;

  protected:
    long binNumber_ = -1;
    DiscretizedActionMap *map_ = nullptr;
    std::unique_ptr<ActionNode> childNode_ = nullptr;
    long visitCount_ = 0;
    double totalQValue_ = 0;
    double meanQValue_ = 0;
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
