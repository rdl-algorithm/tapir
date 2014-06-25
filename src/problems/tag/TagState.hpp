#ifndef TAGSTATE_HPP_
#define TAGSTATE_HPP_

#include <cstddef>                      // for size_t

#include <memory>
#include <ostream>                      // for ostream
#include <vector>

#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/abstract-problem/State.hpp"
#include "solver/abstract-problem/VectorState.hpp"

namespace tag {
class TagState : public solver::VectorState {
    friend class TagTextSerializer;
  public:
    TagState(GridPosition robotPos, GridPosition opponentPos, bool _isTagged);

    virtual ~TagState() = default;
    TagState(TagState const &);
    TagState(TagState &&) = delete;
    virtual TagState &operator=(TagState const &) = delete;
    virtual TagState &operator=(TagState &&) = delete;

    std::unique_ptr<solver::Point> copy() const override;

    double distanceTo(solver::State const &otherState) const override;
    bool equals(solver::State const &otherState) const override;
    std::size_t hash() const;

    std::vector<double> asVector() const override;
    void print(std::ostream &os) const override;

    GridPosition getRobotPosition() const;
    GridPosition getOpponentPosition() const;
    bool isTagged() const;
  private:
    GridPosition robotPos_, opponentPos_;
    bool isTagged_;
};
} /* namespace tag */

// We define a hash function directly in the std namespace.
namespace std {
template<> struct hash<tag::TagState> {
    std::size_t operator()(tag::TagState const &state) const {
        return state.hash();
    }
};
} /* namespace std */

#endif /* TAGSTATE_HPP_ */
