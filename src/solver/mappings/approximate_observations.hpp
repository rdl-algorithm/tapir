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

    virtual std::unique_ptr<ObservationPool> createObservationPool(
            Solver *solver) override;
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

class ApproximateObservationMapEntry;

class ApproximateObservationMap: public solver::ObservationMapping {
  public:
    friend class ApproximateObservationTextSerializer;
    ApproximateObservationMap(double maxDistance);

    // Default destructor; copying and moving disallowed!
    virtual ~ApproximateObservationMap() = default;
    _NO_COPY_OR_MOVE(ApproximateObservationMap);

    /* -------------- Association with an action node ---------------- */
    virtual void setOwner(ActionNode *owner) override;
    virtual ActionNode *getOwner() const override;

    /* -------------- Creation and retrieval of nodes. ---------------- */
    virtual BeliefNode *getBelief(Observation const &obs) const override;
    virtual BeliefNode *createBelief(Observation const &obs) override;

    /* -------------- Retrieval of mapping entries. ---------------- */
    virtual long getNChildren() const override;
    virtual ObservationMappingEntry const *getEntry(Observation const &obs) const override;

    /* --------------- Methods for accessing visit counts. ----------------- */
    virtual void updateVisitCount(Observation const &obs, long deltaNVisits) override;
    virtual long getVisitCount(Observation const &obs) const override;
    virtual long getTotalVisitCount() const override;
  private:
    /* --------------- Private methods to find entries. ----------------- */
    virtual ApproximateObservationMapEntry const *getApproxEntry(Observation const &obs) const;
    virtual ApproximateObservationMapEntry *getApproxEntry(Observation const &obs);


    ActionNode *owningActionNode_;
    double maxDistance_;
    std::vector<std::unique_ptr<ApproximateObservationMapEntry>> entries_;

    long totalVisitCount_;
};

class ApproximateObservationMapEntry : public solver::ObservationMappingEntry {
    friend class ApproximateObservationMap;
    friend class ApproximateObservationTextSerializer;
public:
    virtual ObservationMapping *getMapping() const override;
    virtual std::unique_ptr<Observation> getObservation() const override;
    virtual BeliefNode *getBeliefNode() const override;
    virtual long getVisitCount() const override;
private:
    ApproximateObservationMap *map_ = nullptr;
    std::unique_ptr<Observation> observation_ = nullptr;
    std::unique_ptr<BeliefNode> childNode_ = nullptr;
    long visitCount_ = 0;
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
