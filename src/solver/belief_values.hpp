#ifndef SOLVER_CACHEDVALUE_HPP_
#define SOLVER_CACHEDVALUE_HPP_

#include "global.hpp"

#include "solver/BeliefNode.hpp"

namespace solver {
class BaseCachedValue {
public:
    BaseCachedValue() = default;
    virtual ~BaseCachedValue() = default;
};

template<typename T> class CachedValue : public BaseCachedValue {
public:
    CachedValue(BeliefNode const *node, std::function<T(BeliefNode const *)> f) :
                node_(node),
                function_(f),
                cache_(),
                lastUpdateTime_(-std::numeric_limits<double>::infinity()) {
    }
    _NO_COPY_OR_MOVE(CachedValue);

    virtual T operator()(BeliefNode * node) {
        if (lastUpdateTime_ < node->getTimeOfLastChange()) {
            updateCache();
        }
        function_(node);
        return getCache();
    }

    virtual void updateCache() {
        lastUpdateTime_ = abt::clock_ms();
        cache_ = function_(node_);
    }

    virtual T getCache() {
        return cache_;
    }
private:
    BeliefNode const *node_;
    std::function<T(BeliefNode const *)> function_;
    T cache_;
    double lastUpdateTime_;
};

template<>
inline std::unique_ptr<solver::Point> CachedValue<std::unique_ptr<solver::Point>>::getCache() {
    return cache_->copy();
}
} /* namespace solver */

#endif /* SOLVER_CACHEDVALUE_HPP_ */
