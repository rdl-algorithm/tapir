#ifndef TRACKERTEXTSERIALIZER_HPP_
#define TRACKERTEXTSERIALIZER_HPP_

#include <iosfwd>                       // for istream, ostream
#include <memory>                       // for unique_ptr

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/Observation.hpp"
#include "solver/abstract-problem/State.hpp"

#include "solver/mappings/enumerated_actions.hpp"
#include "solver/mappings/discrete_observations.hpp"

#include "solver/serialization/TextSerializer.hpp"    // for TextSerializer


namespace solver {
class Solver;
}  /* namespace solver */

namespace tracker {
class TrackerTextSerializer: virtual public solver::TextSerializer,
        virtual public solver::DiscretizedActionTextSerializer,
        virtual public solver::DiscreteObservationTextSerializer {
  public:
    TrackerTextSerializer(solver::Solver *solver);
    virtual ~TrackerTextSerializer() = default;
    TrackerTextSerializer(TrackerTextSerializer const &) = delete;
    TrackerTextSerializer(TrackerTextSerializer &&) = delete;
    TrackerTextSerializer &operator=(TrackerTextSerializer const &) = delete;
    TrackerTextSerializer &operator=(TrackerTextSerializer &&) = delete;

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
} /* namespace tracker */

#endif /* TRACKERTEXTSERIALIZER_HPP_ */
