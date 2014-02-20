#ifndef SOLVER_ENUMERATED_ACTIONS_HPP_
#define SOLVER_ENUMERATED_ACTIONS_HPP_

#include "ActionPool.hpp"
#include "ActionMapping.hpp"
#include "Model.hpp"
#include "Serializer.hpp"

#include <memory>
#include <vector>

namespace solver {
class ActionPool;
class ActionNode;
class EnumeratedPoint;

class ModelWithEnumeratedActions : public solver::Model {
public:
    ModelWithEnumeratedActions() = default;
    virtual ~ModelWithEnumeratedActions();
    _NO_COPY_OR_MOVE(ModelWithEnumeratedActions);

    virtual std::unique_ptr<ActionPool> createActionPool();
    virtual std::vector<std::unique_ptr<EnumeratedPoint>> getAllActions() = 0;
};

class EnumeratedActionPool: public solver::ActionPool {
    friend class TextSerializer;
  public:
    EnumeratedActionPool(
            std::vector<std::unique_ptr<EnumeratedPoint>> actions);
    virtual ~EnumeratedActionPool();
    _NO_COPY_OR_MOVE(EnumeratedActionPool);

    virtual std::unique_ptr<ActionMapping> createActionMapping();
private:
  std::vector<std::unique_ptr<EnumeratedPoint>> actions_;
};

class EnumeratedActionMap: public solver::ActionMapping {
  public:
    friend class TextSerializer;
    EnumeratedActionMap(ObservationPool *observationPool,
            std::vector<std::unique_ptr<EnumeratedPoint>> const &actions,
            std::vector<long>);

    // Default destructor; copying and moving disallowed!
    virtual ~EnumeratedActionMap();
    _NO_COPY_OR_MOVE(EnumeratedActionMap);

    virtual BeliefNode *getBelief(Action const &obs) const;
    virtual BeliefNode *createBelief(Action const &obs);

    virtual long getNChildren() const;
    virtual long size() const;

    virtual bool hasActionToTry() const;
    virtual std::unique_ptr<Action> getNextActionToTry(
            RandomGenerator *randGen);

    virtual std::unique_ptr<Action> getSearchAction(
            double exploreCofficient);

    virtual void updateBestValue();
    virtual std::unique_ptr<Action> getBestAction() const;
    virtual double getBestMeanQValue() const;
  private:
    ObservationPool *observationPool_;
    std::vector<std::unique_ptr<EnumeratedPoint>> const &actions_;
    std::vector<std::unique_ptr<ActionNode>> children_;
    long nChildren_;
    std::vector<long> actionOrder_;
    std::vector<long>::const_iterator nextActionIterator_;
};

class EnumeratedActionTextSerializer: virtual public solver::Serializer {
  public:
    EnumeratedActionTextSerializer() = default;
    virtual ~EnumeratedActionTextSerializer() = default;
    _NO_COPY_OR_MOVE(EnumeratedActionTextSerializer);

    virtual void saveActionPool(
            ActionPool const &actionPool, std::ostream &os);
    virtual std::unique_ptr<ActionPool> loadActionPool(
            std::istream &is);
    virtual void saveActionMapping(ActionMapping const &map,
            std::ostream &os);
    virtual std::unique_ptr<ActionMapping> loadActionMapping(
            std::istream &is);
};
} /* namespace solver */

#endif /* SOLVER_ENUMERATED_ACTIONS_HPP_ */
