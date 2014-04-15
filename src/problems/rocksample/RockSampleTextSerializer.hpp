#ifndef ROCKSAMPLE_TEXTSERIALIZER_HPP_
#define ROCKSAMPLE_TEXTSERIALIZER_HPP_

#include <iosfwd>                       // for istream, ostream
#include <memory>                       // for unique_ptr

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/State.hpp"
#include "solver/abstract-problem/Observation.hpp"

#include "solver/mappings/enumerated_actions.hpp"
#include "solver/mappings/enumerated_observations.hpp"

#include "solver/serialization/TextSerializer.hpp"    // for TextSerializer

#include "legal_actions.hpp"
#include "preferred_actions.hpp"
#include "global.hpp"

namespace solver {
class Solver;
} /* namespace solver */

namespace rocksample {
class RockSampleTextSerializer : virtual public LegalActionsTextSerializer,
    virtual public solver::EnumeratedObservationTextSerializer {
  public:
    RockSampleTextSerializer(solver::Solver *solver);
    virtual ~RockSampleTextSerializer() = default;
    _NO_COPY_OR_MOVE(RockSampleTextSerializer);

    void saveState(solver::State const *state, std::ostream &os) override;
    std::unique_ptr<solver::State> loadState(std::istream &is) override;

    void saveObservation(solver::Observation const *obs,
            std::ostream &os) override;
    std::unique_ptr<solver::Observation> loadObservation(
            std::istream &is) override;

    void saveAction(solver::Action const *action, std::ostream &os) override;
    std::unique_ptr<solver::Action> loadAction(std::istream &is) override;


    virtual int getActionColumnWidth() override;
    virtual int getTPColumnWidth() override;
    virtual int getObservationColumnWidth() override;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_TEXTSERIALIZER_HPP_ */
