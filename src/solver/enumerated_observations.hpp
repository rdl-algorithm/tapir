#ifndef SOLVER_ENUMERATED_OBSERVATIONS_HPP_
#define SOLVER_ENUMERATED_OBSERVATIONS_HPP_

#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

#include "Model.hpp"
#include "ObservationPool.hpp"
#include "ObservationMapping.hpp"
#include "Serializer.hpp"

namespace solver {
class ActionPool;
class BeliefNode;
class EnumeratedPoint;

class ModelWithEnumeratedObservations : public solver::Model {
public:
    ModelWithEnumeratedObservations() = default;
    virtual ~ModelWithEnumeratedObservations() = default;
    _NO_COPY_OR_MOVE(ModelWithEnumeratedObservations);

    virtual std::unique_ptr<ObservationPool> createObservationPool();
    virtual std::vector<std::unique_ptr<EnumeratedPoint>>
    getAllObservationsInOrder() = 0;
};

class EnumeratedObservationPool: public solver::ObservationPool {
    friend class TextSerializer;
  public:
    EnumeratedObservationPool(
            std::vector<std::unique_ptr<EnumeratedPoint>> observations);
    virtual ~EnumeratedObservationPool() = default;
    _NO_COPY_OR_MOVE(EnumeratedObservationPool);

    virtual std::unique_ptr<ObservationMapping> createObservationMapping();
private:
  std::vector<std::unique_ptr<EnumeratedPoint>> observations_;
};

class EnumeratedObservationMap: public solver::ObservationMapping {
  public:
    friend class TextSerializer;
    EnumeratedObservationMap(ActionPool *actionPool,
            std::vector<std::unique_ptr<EnumeratedPoint>>
            const &observations);

    // Default destructor; copying and moving disallowed!
    virtual ~EnumeratedObservationMap() = default;
    _NO_COPY_OR_MOVE(EnumeratedObservationMap);

    virtual long size() const;

    virtual BeliefNode *getBelief(Observation const &obs) const;
    virtual BeliefNode *createBelief(Observation const &obs);
  private:
    ActionPool *actionPool_;
    std::vector<std::unique_ptr<EnumeratedPoint>> const &observations_;
    std::vector<std::unique_ptr<BeliefNode>> children_;
};

class EnumeratedObservationTextSerializer: virtual public solver::Serializer {
  public:
    EnumeratedObservationTextSerializer() = default;
    virtual ~EnumeratedObservationTextSerializer() = default;
    _NO_COPY_OR_MOVE(EnumeratedObservationTextSerializer);

    virtual void saveObservationPool(
            ObservationPool const &observationPool, std::ostream &os);
    virtual std::unique_ptr<ObservationPool> loadObservationPool(
            std::istream &is);
    virtual void saveObservationMapping(ObservationMapping const &map,
            std::ostream &os);
    virtual std::unique_ptr<ObservationMapping> loadObservationMapping(
            std::istream &is);
};
} /* namespace solver */

#endif /* SOLVER_ENUMERATED_OBSERVATIONS_HPP_ */
