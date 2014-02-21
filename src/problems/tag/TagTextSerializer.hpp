#ifndef TAGTEXTSERIALIZER_HPP_
#define TAGTEXTSERIALIZER_HPP_

#include <iosfwd>                       // for istream, ostream
#include <memory>                       // for unique_ptr

#include "solver/topology/Action.hpp"
#include "solver/mappings/enumerated_actions.hpp"
#include "solver/mappings/discrete_observations_map.hpp"
#include "solver/topology/State.hpp"
#include "solver/serialization/TextSerializer.hpp"    // for TextSerializer
#include "solver/topology/Observation.hpp"

namespace solver {
class Solver;
}  /* namespace solver */

namespace tag {
class TagTextSerializer: virtual public solver::TextSerializer,
        virtual public solver::EnumeratedActionTextSerializer,
        virtual public solver::DiscreteObservationTextSerializer {
  public:
    TagTextSerializer(solver::Solver *solver);
    virtual ~TagTextSerializer() = default;
    TagTextSerializer(TagTextSerializer const &) = delete;
    TagTextSerializer(TagTextSerializer &&) = delete;
    TagTextSerializer &operator=(TagTextSerializer const &) = delete;
    TagTextSerializer &operator=(TagTextSerializer &&) = delete;

    void saveState(solver::State const *state, std::ostream &os) override;
    std::unique_ptr<solver::State> loadState(std::istream &is) override;

    void saveObservation(solver::Observation const *obs,
            std::ostream &os) override;
    std::unique_ptr<solver::Observation> loadObservation(
            std::istream &is) override;

    void saveAction(solver::Action const *action, std::ostream &os) override;
    std::unique_ptr<solver::Action> loadAction(std::istream &is) override;
};
} /* namespace tag */

#endif /* TAGTEXTSERIALIZER_HPP_ */
