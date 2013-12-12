#ifndef ROCKSAMPLE_TEXTSERIALIZER_HPP_
#define ROCKSAMPLE_TEXTSERIALIZER_HPP_

#include <iosfwd>                       // for istream, ostream
#include <memory>                       // for unique_ptr

#include "solver/TextSerializer.hpp"    // for TextSerializer

namespace solver {
class Solver;
class State;
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

    void saveState(solver::State &state, std::ostream &os);
    std::unique_ptr<solver::State> loadState(std::istream &is);
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_TEXTSERIALIZER_HPP_ */
