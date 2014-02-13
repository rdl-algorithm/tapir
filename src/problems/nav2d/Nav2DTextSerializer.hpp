#ifndef NAV2DTEXTSERIALIZER_HPP_
#define NAV2DTEXTSERIALIZER_HPP_

#include <iosfwd>                       // for istream, ostream
#include <memory>                       // for unique_ptr

#include "solver/TextSerializer.hpp"    // for TextSerializer
#include "solver/State.hpp"

namespace solver {
class Solver;
}  /* namespace solver */

namespace nav2d {
class Nav2DTextSerializer : public solver::TextSerializer {
  public:
    Nav2DTextSerializer();
    Nav2DTextSerializer(solver::Solver *solver);
    virtual ~Nav2DTextSerializer() = default;
    Nav2DTextSerializer(Nav2DTextSerializer const &) = delete;
    Nav2DTextSerializer(Nav2DTextSerializer &&) = delete;
    Nav2DTextSerializer &operator=(Nav2DTextSerializer const &) = delete;
    Nav2DTextSerializer &operator=(Nav2DTextSerializer &&) = delete;

    void saveState(solver::State const &state, std::ostream &os);
    std::unique_ptr<solver::State> loadState(std::istream &is);
};
} /* namespace nav2d */

#endif /* NAV2DTEXTSERIALIZER_HPP_ */
