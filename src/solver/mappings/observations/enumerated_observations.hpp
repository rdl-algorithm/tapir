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

class EnumeratedObservationPool: public solver::ObservationPool {
  public:
    EnumeratedObservationPool(std::vector<std::unique_ptr<DiscretizedPoint>> observations);
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
    EnumeratedObservationMap(
            std::vector<std::unique_ptr<DiscretizedPoint>> const &allObservations);

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
    virtual ObservationMappingEntry *getEntry(Observation const &obs) override;
    virtual ObservationMappingEntry const *getEntry(Observation const &obs) const override;
    virtual std::vector<ObservationMappingEntry const *> getAllEntries() const override;

    /* ------------- Methods for accessing visit counts. --------------- */
    virtual long getTotalVisitCount() const override;
  private:
    ActionNode *owningActionNode_;
    std::vector<std::unique_ptr<DiscretizedPoint>> const &allObservations_;
    std::vector<std::unique_ptr<EnumeratedObservationMapEntry>> children_;

    long nChildren_;
    long totalVisitCount_;
};

class EnumeratedObservationMapEntry : public solver::ObservationMappingEntry {
    friend class EnumeratedObservationMap;
    friend class EnumeratedObservationTextSerializer;
public:
    virtual ObservationMapping *getMapping() const override;
    virtual std::unique_ptr<Observation> getObservation() const override;
    virtual BeliefNode *getBeliefNode() const override;
    virtual long getVisitCount() const override;

    virtual void updateVisitCount(long deltaNVisits) override;
private:
    EnumeratedObservationMap *map_ = nullptr;
    long index_ = 0;
    std::unique_ptr<BeliefNode> childNode_ = nullptr;
    long visitCount_ = 0;
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
