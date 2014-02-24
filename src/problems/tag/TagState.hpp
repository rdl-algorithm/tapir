#ifndef TAGSTATE_HPP_
#define TAGSTATE_HPP_

#include <cstddef>                      // for size_t

#include <memory>
#include <ostream>                      // for ostream
#include <vector>

#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/geometry/State.hpp"
#include "solver/geometry/Vector.hpp"             // for State

namespace tag {
class TagState : public solver::Vector {
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

#endif /* TAGSTATE_HPP_ */
