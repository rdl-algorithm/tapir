#ifndef UNDERWATERNAVTEXTSERIALIZER_HPP_
#define UNDERWATERNAVTEXTSERIALIZER_HPP_

#include <iosfwd>                       // for istream, ostream
#include <memory>                       // for unique_ptr

#include "solver/TextSerializer.hpp"    // for TextSerializer

namespace solver {
class Solver;
class State;
}  /* namespace solver */

namespace uwnav {
class UnderwaterNavTextSerializer : public solver::TextSerializer {
  public:
    UnderwaterNavTextSerializer();
    UnderwaterNavTextSerializer(solver::Solver *solver);
    virtual ~UnderwaterNavTextSerializer() = default;
    UnderwaterNavTextSerializer(UnderwaterNavTextSerializer const &) = delete;
    UnderwaterNavTextSerializer(UnderwaterNavTextSerializer &&) = delete;
    UnderwaterNavTextSerializer &operator=(UnderwaterNavTextSerializer const &) = delete;
    UnderwaterNavTextSerializer &operator=(UnderwaterNavTextSerializer &&) = delete;

    void saveState(solver::State &state, std::ostream &os);
    std::unique_ptr<solver::State> loadState(std::istream &is);
};
} /* namespace uwnav */

#endif /* UNDERWATERNAVTEXTSERIALIZER_HPP_ */
