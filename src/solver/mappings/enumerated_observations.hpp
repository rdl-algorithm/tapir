#ifndef SOLVER_ENUMERATED_OBSERVATIONS_HPP_
#define SOLVER_ENUMERATED_OBSERVATIONS_HPP_

#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

#include "solver/BeliefNode.hpp"

#include "solver/abstract-problem/Model.hpp"

#include "solver/serialization/Serializer.hpp"

#include "ObservationPool.hpp"
#include "ObservationMapping.hpp"

namespace solver {
class ActionPool;
class DiscretizedPoint;
class EnumeratedObservationMapEntry;

class ModelWithEnumeratedObservations : virtual public solver::Model {
public:
    ModelWithEnumeratedObservations() = default;
    virtual ~ModelWithEnumeratedObservations() = default;
    _NO_COPY_OR_MOVE(ModelWithEnumeratedObservations);

    virtual std::unique_ptr<ObservationPool> createObservationPool(
            Solver *solver) override;
    virtual std::vector<std::unique_ptr<DiscretizedPoint>>
    getAllObservationsInOrder() = 0;
};

class EnumeratedObservationPool: public solver::ObservationPool {
  public:
    EnumeratedObservationPool(Solver *solver,
            std::vector<std::unique_ptr<DiscretizedPoint>> observations);
    virtual ~EnumeratedObservationPool() = default;
    _NO_COPY_OR_MOVE(EnumeratedObservationPool);

    virtual std::unique_ptr<ObservationMapping> createObservationMapping() override;
private:
  std::vector<std::unique_ptr<DiscretizedPoint>> observations_;
};

class EnumeratedObservationMap: public solver::ObservationMapping {
  public:
    friend class EnumeratedObservationMapEntry;
    friend class EnumeratedObservationTextSerializer;
    EnumeratedObservationMap(ActionPool *actionPool,
            std::vector<std::unique_ptr<DiscretizedPoint>>
            const &allObservations);

    // Default destructor; copying and moving disallowed!
    virtual ~EnumeratedObservationMap() = default;
    _NO_COPY_OR_MOVE(EnumeratedObservationMap);

    /* -------------- Association with an action node ---------------- */
    virtual void setOwner(ActionNode *owner) override;
    virtual ActionNode *getOwner() const override;

    /* -------------- Creation and retrieval of nodes. ---------------- */
    virtual BeliefNode *getBelief(Observation const &obs) const override;
    virtual BeliefNode *createBelief(Observation const &obs) override;

    /* -------------- Retrieval of mapping entries. ---------------- */
    virtual long getNChildren() const override;
    virtual ObservationMappingEntry const *getEntry(Observation const &obs) const override;

    /* ------------- Methods for accessing visit counts. --------------- */
    virtual void updateVisitCount(Observation const &obs, long deltaNVisits) override;
    virtual long getVisitCount(Observation const &obs) const override;
    virtual long getTotalVisitCount() const override;
  private:
    ActionNode *owningActionNode_;
    std::vector<std::unique_ptr<DiscretizedPoint>> const &allObservations_;
    ActionPool *actionPool_;
    std::vector<std::unique_ptr<EnumeratedObservationMapEntry>> children_;

    long nChildren_;
    long totalVisitCount_;
};

class EnumeratedObservationMapEntry : public solver::ObservationMappingEntry {
    friend class EnumeratedObservationMap;
    friend class EnumeratedObservationTextSerializer;
public:
    EnumeratedObservationMapEntry(EnumeratedObservationMap *map,
            long index, std::unique_ptr<BeliefNode> childNode);
    virtual ~EnumeratedObservationMapEntry() = default;
    _NO_COPY_OR_MOVE(EnumeratedObservationMapEntry);

    virtual ObservationMapping *getMapping() const override;
    virtual std::unique_ptr<Observation> getObservation() const override;
    virtual BeliefNode *getBeliefNode() const override;
    virtual long getVisitCount() const override;
private:
    EnumeratedObservationMap *map_;
    long index_;
    std::unique_ptr<BeliefNode> childNode_;
    long visitCount_;
};

class EnumeratedObservationTextSerializer: virtual public solver::Serializer {
  public:
    EnumeratedObservationTextSerializer() = default;
    virtual ~EnumeratedObservationTextSerializer() = default;
    _NO_COPY_OR_MOVE(EnumeratedObservationTextSerializer);

    virtual void saveObservationPool(
            ObservationPool const &observationPool, std::ostream &os) override;
    virtual std::unique_ptr<ObservationPool> loadObservationPool(
            std::istream &is) override;
    virtual void saveObservationMapping(ObservationMapping const &map,
            std::ostream &os) override;
    virtual std::unique_ptr<ObservationMapping> loadObservationMapping(
            std::istream &is) override;
};
} /* namespace solver */

#endif /* SOLVER_ENUMERATED_OBSERVATIONS_HPP_ */
