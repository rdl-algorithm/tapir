#ifndef SPATIALINDEXQUERY_HPP_
#define SPATIALINDEXQUERY_HPP_

#include <unordered_set>

#include <spatialindex/SpatialIndex.h>

#include "StatePool.hpp"
#include "StateQuery.hpp"
#include "StateSpatialIndex.hpp"

namespace SpatialIndex {
class ISpatialIndex;
}

namespace solver {
class VectorState;

class SpatialIndexQuery: public StateQuery {
public:
    SpatialIndexQuery(unsigned long nDim_, StatePool *statePool,
            SpatialIndex::ISpatialIndex *spatialIndex);
    virtual ~SpatialIndexQuery() = default;
    SpatialIndexQuery(SpatialIndexQuery const &) = delete;
    SpatialIndexQuery(SpatialIndexQuery &&) = delete;
    virtual SpatialIndexQuery &operator=(SpatialIndexQuery const &) = delete;
    virtual SpatialIndexQuery &operator=(SpatialIndexQuery &&) = delete;

    virtual StateInfoSet getStates();
    virtual void clearStates();

    virtual void markStates(VectorState *corner0, VectorState *corner1);

    class Visitor : public SpatialIndex::IVisitor {
    public:
        Visitor(SpatialIndexQuery *query);
        virtual ~Visitor() = default;
        Visitor(Visitor const &) = delete;
        Visitor(Visitor &&) = delete;
        virtual Visitor &operator=(Visitor const &) = delete;
        virtual Visitor &operator=(Visitor &&) = delete;

        virtual void visitNode(const SpatialIndex::INode &node);
        virtual void visitData(const SpatialIndex::IData &data);
        virtual void visitData(std::vector<const SpatialIndex::IData *> &v);
    private:
        SpatialIndexQuery *query_;
    };
private:
    unsigned long nDim_;
    StatePool *statePool_;
    SpatialIndex::ISpatialIndex *spatialIndex_;
    StateInfoSet states_;
};

} /* namespace solver */

#endif /* SPATIALINDEXQUERY_HPP_ */
