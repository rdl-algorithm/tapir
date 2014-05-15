#ifndef SOLVER_AVERAGEQMAXCHILD_HPP_
#define SOLVER_AVERAGEQMAXCHILD_HPP_

namespace solver {

class AverageQMaxChild: public BeliefQValueEstimator {
public:
    AverageQMaxChild(ActionMapping *mapping);
    virtual ~AverageQMaxChild() = default;

    /** Recalculates the q-value as the average q-value of its action children (weighted by
     *  the visit counts), and sets the recommended action as the action with the highest q-value.
     */
    virtual void recalculate() override;

    /** Return the average q-value for the belief, at the time of last recalculation. */
    virtual double getBeliefQValue() const override;

    /** Return the action with the highest q-value at the time of last recalculation. */
    virtual std::unique_ptr<Action> getRecommendedAction() const override;

private:
    ActionMapping *mapping_;
};
} /* namespace solver */

#endif /* SOLVER_AVERAGEQMAXCHILD_HPP_ */
