#ifndef ROCKSAMPLETEXTSERIALIZER_HPP
#define ROCKSAMPLETEXTSERIALIZER_HPP

#include <iosfwd>                       // for istream, ostream
#include <memory>                       // for unique_ptr

#include "solver/TextSerializer.hpp"    // for TextSerializer

class Solver;
class State;

class RockSampleTextSerializer : public TextSerializer {
  public:
    RockSampleTextSerializer();
    RockSampleTextSerializer(Solver *solver);
    virtual ~RockSampleTextSerializer() = default;
    RockSampleTextSerializer(RockSampleTextSerializer const &) = delete;
    RockSampleTextSerializer(RockSampleTextSerializer &&) = delete;
    RockSampleTextSerializer &operator=(RockSampleTextSerializer const &) =
        delete;
    RockSampleTextSerializer &operator=(RockSampleTextSerializer &&) = delete;

    void saveState(State &state, std::ostream &os);
    std::unique_ptr<State> loadState(std::istream &is);
};

#endif /* ROCKSAMPLETEXTSERIALIZER_HPP */
