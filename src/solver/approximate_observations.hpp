#ifndef SOLVER_APPROXIMATE_OBSERVATIONS_HPP_
#define SOLVER_APPROXIMATE_OBSERVATIONS_HPP_

#include "Model.hpp"
#include "ObservationPool.hpp"
#include "ObservationMapping.hpp"
#include "Serializer.hpp"

#include <memory>
#include <vector>

namespace solver {
class ActionPool;
class BeliefNode;

class ModelWithApproximateObservations : virtual public solver::Model {
public:
    ModelWithApproximateObservations();
    virtual ~ModelWithApproximateObservations();
    _NO_COPY_OR_MOVE(ModelWithApproximateObservations);

    virtual std::unique_ptr<ObservationPool> createObservationPool() override;
    virtual double getMaxObservationDistance() = 0;
};

class ApproximateObservationPool: public solver::ObservationPool {
    friend class TextSerializer;
  public:
    ApproximateObservationPool(double maxDistance);
    virtual ~ApproximateObservationPool();
    _NO_COPY_OR_MOVE(ApproximateObservationPool);

    virtual std::unique_ptr<ObservationMapping>
    createObservationMapping() override;
  private:
    double maxDistance_;
};

class ApproximateObservationMap: public solver::ObservationMapping {
  public:
    friend class TextSerializer;
    friend class ApproximateObservationTextSerializer;
    ApproximateObservationMap(ActionPool *actionPool, double maxDistance);

    // Default destructor; copying and moving disallowed!
    virtual ~ApproximateObservationMap();
    _NO_COPY_OR_MOVE(ApproximateObservationMap);

    virtual BeliefNode *getBelief(Observation const &obs) const override;
    virtual BeliefNode *createBelief(Observation const &obs) override;
  private:
    ActionPool *actionPool_;
    double maxDistance_;
    typedef std::pair<std::unique_ptr<Observation>,
            std::unique_ptr<BeliefNode>> Entry;
    typedef std::vector<Entry> ChildMappingVector;
    ChildMappingVector children_;
};


class ApproximateObservationTextSerializer: virtual public solver::Serializer {
  public:
    ApproximateObservationTextSerializer() = default;
    virtual ~ApproximateObservationTextSerializer();
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
