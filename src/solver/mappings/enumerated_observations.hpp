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

class ModelWithEnumeratedObservations : virtual public solver::Model {
public:
    ModelWithEnumeratedObservations() = default;
    virtual ~ModelWithEnumeratedObservations() = default;
    _NO_COPY_OR_MOVE(ModelWithEnumeratedObservations);

    virtual std::unique_ptr<ObservationPool> createObservationPool() override;
    virtual std::vector<std::unique_ptr<DiscretizedPoint>>
    getAllObservationsInOrder() = 0;
};

class EnumeratedObservationPool: public solver::ObservationPool {
  public:
    EnumeratedObservationPool(
            std::vector<std::unique_ptr<DiscretizedPoint>> observations);
    virtual ~EnumeratedObservationPool() = default;
    _NO_COPY_OR_MOVE(EnumeratedObservationPool);

    virtual std::unique_ptr<ObservationMapping> createObservationMapping() override;
private:
  std::vector<std::unique_ptr<DiscretizedPoint>> observations_;
};

struct EnumeratedObservationMapEntry {
    std::unique_ptr<BeliefNode> childNode = nullptr;
    long visitCount = 0;
};

class EnumeratedObservationMap: public solver::ObservationMapping {
  public:
    friend class EnumeratedObservationTextSerializer;
    EnumeratedObservationMap(ActionPool *actionPool,
            std::vector<std::unique_ptr<DiscretizedPoint>>
            const &allObservations);

    // Default destructor; copying and moving disallowed!
    virtual ~EnumeratedObservationMap() = default;
    _NO_COPY_OR_MOVE(EnumeratedObservationMap);

    virtual BeliefNode *getBelief(Observation const &obs) const override;
    virtual BeliefNode *createBelief(Observation const &obs) override;

    virtual long getNChildren() const override;
    virtual long size() const;

    virtual void updateVisitCount(Observation const &obs, long deltaNVisits) override;
    virtual long getVisitCount(Observation const &obs) const override;
    virtual long getTotalVisitCount() const override;
  private:
    std::vector<std::unique_ptr<DiscretizedPoint>> const &allObservations_;
    ActionPool *actionPool_;
    std::vector<EnumeratedObservationMapEntry> children_;

    long nChildren_;
    long totalVisitCount_;
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
