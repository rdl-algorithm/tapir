#ifndef NAV2D_TEXTSERIALIZER_HPP_
#define NAV2D_TEXTSERIALIZER_HPP_

#include <iosfwd>                       // for istream, ostream
#include <memory>                       // for unique_ptr

#include "solver/geometry/Action.hpp"
#include "solver/geometry/State.hpp"
#include "solver/geometry/Observation.hpp"

#include "solver/mappings/enumerated_actions.hpp"
#include "solver/mappings/enumerated_observations.hpp"

#include "solver/serialization/TextSerializer.hpp"    // for TextSerializer

#include "global.hpp"

namespace solver {
class Solver;
} /* namespace solver */

namespace nav2d {
class Nav2DTextSerializer : virtual public solver::TextSerializer,
    virtual public solver::EnumeratedActionTextSerializer,
    virtual public solver::EnumeratedObservationTextSerializer {
  public:
    Nav2DTextSerializer(solver::Solver *solver);
    virtual ~Nav2DTextSerializer() = default;
    _NO_COPY_OR_MOVE(Nav2DTextSerializer);

    void saveState(solver::State const *state, std::ostream &os) override;
    std::unique_ptr<solver::State> loadState(std::istream &is) override;

    void saveObservation(solver::Observation const *obs,
            std::ostream &os) override;
    std::unique_ptr<solver::Observation> loadObservation(
            std::istream &is) override;

    void saveAction(solver::Action const *action, std::ostream &os) override;
    std::unique_ptr<solver::Action> loadAction(std::istream &is) override;
};
} /* namespace nav2d */

#endif /* NAV2D_TEXTSERIALIZER_HPP_ */
