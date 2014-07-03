#ifndef HOMECARETEXTSERIALIZER_HPP_
#define HOMECARETEXTSERIALIZER_HPP_

#include <iosfwd>                       // for istream, ostream
#include <memory>                       // for unique_ptr

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/ModelChange.hpp"
#include "solver/abstract-problem/Observation.hpp"
#include "solver/abstract-problem/State.hpp"

#include "solver/mappings/actions/enumerated_actions.hpp"
#include "solver/mappings/observations/discrete_observations.hpp"

#include "solver/serialization/TextSerializer.hpp"    // for TextSerializer


namespace solver {
class Solver;
}  /* namespace solver */

namespace homecare {
class HomecareTextSerializer: virtual public solver::TextSerializer,
        virtual public solver::EnumeratedActionTextSerializer,
        virtual public solver::DiscreteObservationTextSerializer {
  public:
    HomecareTextSerializer(solver::Solver *solver);
    virtual ~HomecareTextSerializer() = default;
    HomecareTextSerializer(HomecareTextSerializer const &) = delete;
    HomecareTextSerializer(HomecareTextSerializer &&) = delete;
    HomecareTextSerializer &operator=(HomecareTextSerializer const &) = delete;
    HomecareTextSerializer &operator=(HomecareTextSerializer &&) = delete;

    /* ------------------ Saving change sequences -------------------- */
    virtual void saveModelChange(solver::ModelChange const &change, std::ostream &os) override;
    virtual std::unique_ptr<solver::ModelChange> loadModelChange(std::istream &is) override;


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
} /* namespace homecare */

#endif /* HOMECARETEXTSERIALIZER_HPP_ */
