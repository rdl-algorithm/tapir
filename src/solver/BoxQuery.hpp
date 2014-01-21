#ifndef SPATIALINDEXQUERY_HPP_
#define SPATIALINDEXQUERY_HPP_

#include <unordered_set>

#include <spatialindex/SpatialIndex.h>

#include "StatePool.hpp"
#include "StateIndexQuery.hpp"
#include "StateIndex.hpp"

namespace SpatialIndex {
class ISpatialIndex;
}

namespace solver {
class VectorState;

class BoxQuery: public StateIndexQuery {
public:
    BoxQuery(unsigned long nSDim, StatePool *statePool,
            SpatialIndex::ISpatialIndex *spatialIndex);
    virtual ~BoxQuery() = default;
    BoxQuery(BoxQuery const &) = default;
    BoxQuery(BoxQuery &&) = default;
    virtual BoxQuery &operator=(BoxQuery const &) = default;
    virtual BoxQuery &operator=(BoxQuery &&) = default;

    virtual StateInfoSet getStates();
    virtual void clearStates();

    virtual void markStates(std::vector<double> lowCorner,
            std::vector<double> highCorner);

    class Visitor : public SpatialIndex::IVisitor {
    public:
        Visitor(BoxQuery *query);
        virtual ~Visitor() = default;
        Visitor(Visitor const &) = delete;
        Visitor(Visitor &&) = delete;
        virtual Visitor &operator=(Visitor const &) = delete;
        virtual Visitor &operator=(Visitor &&) = delete;

        virtual void visitNode(const SpatialIndex::INode &node);
        virtual void visitData(const SpatialIndex::IData &data);
        virtual void visitData(std::vector<const SpatialIndex::IData *> &v);
    private:
        BoxQuery *query_;
    };
private:
    unsigned long nSDim_;
    StatePool *statePool_;
    SpatialIndex::ISpatialIndex *spatialIndex_;
    StateInfoSet states_;
};

} /* namespace solver */

#endif /* SPATIALINDEXQUERY_HPP_ */
