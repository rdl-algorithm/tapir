#ifndef NAV2D_TEXTSERIALIZER_HPP_
#define NAV2D_TEXTSERIALIZER_HPP_

#include <iosfwd>                       // for istream, ostream
#include <memory>                       // for unique_ptr

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/State.hpp"
#include "solver/abstract-problem/Observation.hpp"
#include "solver/abstract-problem/TransitionParameters.hpp"

#include "solver/mappings/actions/discretized_actions.hpp"
#include "solver/mappings/observations/approximate_observations.hpp"

#include "solver/serialization/TextSerializer.hpp"    // for TextSerializer

#include "global.hpp"

namespace solver {
class Solver;
} /* namespace solver */

namespace nav2d {
class Nav2DTextSerializer: virtual public solver::TextSerializer,
        virtual public solver::DiscretizedActionTextSerializer,
        virtual public solver::ApproximateObservationTextSerializer {
public:
    Nav2DTextSerializer(solver::Solver *solver);
    virtual ~Nav2DTextSerializer() = default;
    _NO_COPY_OR_MOVE(Nav2DTextSerializer);

    /* ------------------ Saving change sequences -------------------- */
    virtual void saveModelChange(solver::ModelChange const &change,
            std::ostream &os) override;
    virtual std::unique_ptr<solver::ModelChange> loadModelChange(
            std::istream &is) override;

    void saveState(solver::State const *state, std::ostream &os) override;
    std::unique_ptr<solver::State> loadState(std::istream &is) override;

    void saveAction(solver::Action const *action, std::ostream &os) override;
    std::unique_ptr<solver::Action> loadAction(std::istream &is) override;

    virtual void saveTransitionParameters(
            solver::TransitionParameters const *tp, std::ostream &os) override;
    virtual std::unique_ptr<solver::TransitionParameters>
    loadTransitionParameters(std::istream &is) override;

    void saveObservation(solver::Observation const *obs, std::ostream &os)
            override;
    std::unique_ptr<solver::Observation> loadObservation(std::istream &is)
            override;

    virtual int getActionColumnWidth() override;
    virtual int getTPColumnWidth() override;
    virtual int getObservationColumnWidth() override;
};
} /* namespace nav2d */

#endif /* NAV2D_TEXTSERIALIZER_HPP_ */
