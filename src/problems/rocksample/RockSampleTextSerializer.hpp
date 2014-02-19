#ifndef ROCKSAMPLE_TEXTSERIALIZER_HPP_
#define ROCKSAMPLE_TEXTSERIALIZER_HPP_

#include <iosfwd>                       // for istream, ostream
#include <memory>                       // for unique_ptr

#include "solver/Action.hpp"
#include "solver/State.hpp"
#include "solver/TextSerializer.hpp"    // for TextSerializer
#include "solver/Observation.hpp"

namespace solver {
class Solver;
} /* namespace solver */

namespace rocksample {
class RockSampleTextSerializer : public solver::TextSerializer {
  public:
    RockSampleTextSerializer();
    RockSampleTextSerializer(solver::Solver *solver);
    virtual ~RockSampleTextSerializer() = default;
    RockSampleTextSerializer(RockSampleTextSerializer const &) = delete;
    RockSampleTextSerializer(RockSampleTextSerializer &&) = delete;
    RockSampleTextSerializer &operator=(RockSampleTextSerializer const &) =
        delete;
    RockSampleTextSerializer &operator=(RockSampleTextSerializer &&) = delete;

    void saveState(solver::State const *state, std::ostream &os);
    std::unique_ptr<solver::State> loadState(std::istream &is);

    void saveObservation(solver::Observation const *obs, std::ostream &os);
    std::unique_ptr<solver::Observation> loadObservation(std::istream &is);

    void saveAction(solver::Action const *action, std::ostream &os);
    std::unique_ptr<solver::Action> loadAction(std::istream &is);
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_TEXTSERIALIZER_HPP_ */
