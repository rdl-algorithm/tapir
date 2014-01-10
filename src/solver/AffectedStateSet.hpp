#ifndef AFFECTEDSTATESET_HPP_
#define AFFECTEDSTATESET_HPP_

namespace solver {
class AffectedStateSet {
  public:
    AffectedStateSet() = default;
    virtual ~AffectedStateSet() = default;
    AffectedStateSet(AffectedStateSet const &) = delete;
    AffectedStateSet(AffectedStateSet &&) = delete;
    virtual AffectedStateSet &operator=(AffectedStateSet const &) = delete;
    virtual AffectedStateSet &operator=(AffectedStateSet &&) = delete;

    virtual void addStateInfo(StateInfo *stateInfo);

    virtual std::unordered_set<StateInfo *> getAffectedStates() = 0;
    virtual void clearAffectedStates() = 0;

    /* For implementation, make additional methods for marking which states
     * are affected - this will be implementation-dependent.
     */
};

}

#endif /* AFFECTEDSTATESET_HPP_ */
