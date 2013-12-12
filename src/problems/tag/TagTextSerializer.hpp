#ifndef TAGTEXTSERIALIZER_HPP_
#define TAGTEXTSERIALIZER_HPP_

#include <iosfwd>                       // for istream, ostream
#include <memory>                       // for unique_ptr

#include "solver/TextSerializer.hpp"    // for TextSerializer

namespace solver {
class Solver;
class State;
}  /* namespace solver */

namespace tag {
class TagTextSerializer : public solver::TextSerializer {
  public:
    TagTextSerializer();
    TagTextSerializer(solver::Solver *solver);
    virtual ~TagTextSerializer() = default;
    TagTextSerializer(TagTextSerializer const &) = delete;
    TagTextSerializer(TagTextSerializer &&) = delete;
    TagTextSerializer &operator=(TagTextSerializer const &) = delete;
    TagTextSerializer &operator=(TagTextSerializer &&) = delete;

    void saveState(solver::State &state, std::ostream &os);
    std::unique_ptr<solver::State> loadState(std::istream &is);
};
} /* namespace tag */

#endif /* TAGTEXTSERIALIZER_HPP_ */
