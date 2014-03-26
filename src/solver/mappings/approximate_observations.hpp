#ifndef SOLVER_APPROXIMATE_OBSERVATIONS_HPP_
#define SOLVER_APPROXIMATE_OBSERVATIONS_HPP_

#include "solver/BeliefNode.hpp"

#include "solver/abstract-problem/Model.hpp"

#include "solver/serialization/Serializer.hpp"

#include "ObservationPool.hpp"
#include "ObservationMapping.hpp"

#include <memory>
#include <vector>

namespace solver {
class ActionPool;
class BeliefNode;

class ModelWithApproximateObservations : virtual public solver::Model {
public:
    ModelWithApproximateObservations() = default;
    virtual ~ModelWithApproximateObservations() = default;
    _NO_COPY_OR_MOVE(ModelWithApproximateObservations);

    virtual std::unique_ptr<ObservationPool> createObservationPool() override;
    virtual double getMaxObservationDistance() = 0;
};

class ApproximateObservationPool: public solver::ObservationPool {
  public:
    ApproximateObservationPool(double maxDistance);
    virtual ~ApproximateObservationPool() = default;
    _NO_COPY_OR_MOVE(ApproximateObservationPool);

    virtual std::unique_ptr<ObservationMapping>
    createObservationMapping() override;
  private:
    double maxDistance_;
};

struct ApproximateObservationMapEntry {
    std::unique_ptr<Observation> observation = nullptr;
    std::unique_ptr<BeliefNode> childNode = nullptr;
    long visitCount = 0;
};

class ApproximateObservationMap: public solver::ObservationMapping {
  public:
    friend class ApproximateObservationTextSerializer;
    ApproximateObservationMap(ActionPool *actionPool, double maxDistance);

    // Default destructor; copying and moving disallowed!
    virtual ~ApproximateObservationMap() = default;
    _NO_COPY_OR_MOVE(ApproximateObservationMap);

    virtual BeliefNode *getBelief(Observation const &obs) const override;
    virtual BeliefNode *createBelief(Observation const &obs) override;

    virtual long getNChildren() const override;

    virtual void updateVisitCount(Observation const &obs, long deltaNVisits) override;
    virtual long getVisitCount(Observation const &obs) const override;
    virtual long getTotalVisitCount() const override;
  private:
    ApproximateObservationMapEntry const *getEntry(Observation const &obs) const;
    ApproximateObservationMapEntry *getEntry(Observation const &obs);

    ActionPool *actionPool_;
    double maxDistance_;
    std::vector<ApproximateObservationMapEntry> children_;

    long totalVisitCount_;
};


class ApproximateObservationTextSerializer: virtual public solver::Serializer {
  public:
    ApproximateObservationTextSerializer() = default;
    virtual ~ApproximateObservationTextSerializer() = default;
    _NO_COPY_OR_MOVE(ApproximateObservationTextSerializer);

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

#endif /* SOLVER_APPROXIMATE_OBSERVATIONS_HPP_ */
