#ifndef SOLVER_DISCRETEACTIONMAP_HPP_
#define SOLVER_DISCRETEACTIONMAP_HPP_

#include <cstddef>

#include <initializer_list>
#include <memory>
#include <utility>
#include <unordered_map>
#include <vector>

#include "Action.hpp"
#include "ActionMapping.hpp"
#include "RandomAccessSet.hpp"
#include "global.hpp"

namespace solver {
class ActionNode;
class BeliefNode;
class Model;

class DiscreteActionMap : public ActionMapping {
public:
    friend class TextSerializer;
    DiscreteActionMap(Model *model_);
    DiscreteActionMap(Model *model_, std::vector<std::unique_ptr<Action>>);

    // Default destructor; copying and moving disallowed!
    ~DiscreteActionMap();
    DiscreteActionMap(DiscreteActionMap const &) = delete;
    DiscreteActionMap(DiscreteActionMap &&) = delete;
    DiscreteActionMap &operator=(DiscreteActionMap const &) = delete;
    DiscreteActionMap &operator=(DiscreteActionMap &&) = delete;

    virtual ActionNode *getActionNode(Action const &action) const;
    /** Creates a new action node for the given action. */
    virtual ActionNode *createActionNode(Action const &action);

    /** Returns the number of distinct action nodes used within this mapping. */
    virtual long size() const;

    /** Returns true iff there is another action that needs to be explored. */
    virtual bool hasActionToTry() const;
    /** Returns the next action to attempt, if any. */
    virtual std::unique_ptr<Action> getNextActionToTry(
            RandomGenerator *randGen);

    /** Chooses a next action with the UCB algorithm. */
    virtual std::unique_ptr<Action> getSearchAction(double ucbExploreCoefficient);

    /** Updates the calculation of which action is optimal. */
    virtual void updateBestValue();
    /** Chooses the action with the best expected value */
    virtual std::unique_ptr<Action> getBestAction() const;
    /** Returns the best q-value */
    virtual double getBestMeanQValue() const;

private:
    Model *model_;

    std::vector<std::unique_ptr<Action>> actions_;
    abt::RandomAccessSet<Action const *> actionsToTry_;

    std::unique_ptr<Action> bestAction_;
    double bestMeanQValue_;

    struct HashContents {
        std::size_t operator()(Action const *obs) const {
            return obs->hash();
        }
    };
    struct EqualContents {
        std::size_t operator()(Action const *o1,
                Action const *o2) const {
            return o1->equals(*o2);
        }
    };
    typedef std::unordered_map<Action const *,
            std::unique_ptr<ActionNode>, HashContents, EqualContents> ActionMap;
    ActionMap actionMap_;
};
} /* namespace solver */

#endif /* SOLVER_DISCRETEACTIONMAP_HPP_ */
