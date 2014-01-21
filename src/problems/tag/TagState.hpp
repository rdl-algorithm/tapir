#ifndef TAGSTATE_HPP_
#define TAGSTATE_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>

#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/VectorState.hpp"             // for State

namespace tag {
class TagState : public solver::VectorState {
    friend class TagTextSerializer;
  public:
    TagState(GridPosition robotPos, GridPosition opponentPos, bool _isTagged);

    virtual ~TagState() = default;
    TagState(TagState const &other);
    TagState(TagState &&) = delete;
    virtual TagState &operator=(TagState const &) = delete;
    virtual TagState &operator=(TagState &&) = delete;

    double distanceTo(solver::State const &otherState) const;
    bool equals(solver::State const &otherState) const;
    std::size_t hash() const;

    std::vector<double> asVector() const;
    void print(std::ostream &os) const;

    GridPosition getRobotPosition() const {
        return robotPos_;
    }

    GridPosition getOpponentPosition() const {
        return opponentPos_;
    }

    bool isTagged() const {
        return isTagged_;
    }

  private:
    GridPosition robotPos_, opponentPos_;
    bool isTagged_;
};
} /* namespace tag */

#endif /* TAGSTATE_HPP_ */
