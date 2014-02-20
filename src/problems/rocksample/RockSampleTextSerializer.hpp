#ifndef ROCKSAMPLE_TEXTSERIALIZER_HPP_
#define ROCKSAMPLE_TEXTSERIALIZER_HPP_

#include <iosfwd>                       // for istream, ostream
#include <memory>                       // for unique_ptr

#include "solver/Action.hpp"
#include "solver/enumerated_actions.hpp"
#include "solver/enumerated_observations.hpp"
#include "solver/State.hpp"
#include "solver/TextSerializer.hpp"    // for TextSerializer
#include "solver/Observation.hpp"

#include "global.hpp"

namespace solver {
class Solver;
} /* namespace solver */

namespace rocksample {
class RockSampleTextSerializer : virtual public solver::TextSerializer,
    virtual public solver::EnumeratedActionTextSerializer,
    virtual public solver::EnumeratedObservationTextSerializer {
  public:
    RockSampleTextSerializer();
    RockSampleTextSerializer(solver::Solver *solver);
    _NO_COPY_OR_MOVE(RockSampleTextSerializer);

    void saveState(solver::State const *state, std::ostream &os);
    std::unique_ptr<solver::State> loadState(std::istream &is);

    void saveObservation(solver::Observation const *obs, std::ostream &os);
    std::unique_ptr<solver::Observation> loadObservation(std::istream &is);

    void saveAction(solver::Action const *action, std::ostream &os);
    std::unique_ptr<solver::Action> loadAction(std::istream &is);
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_TEXTSERIALIZER_HPP_ */
