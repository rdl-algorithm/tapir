#ifndef TAGSTATE_HPP
#define TAGSTATE_HPP

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream

#include "problems/GridPosition.hpp"    // for GridPosition
#include "solver/State.hpp"             // for State

class TagState : public State {
    friend class TagTextSerializer;
  public:
    TagState(GridPosition robotPos, GridPosition opponentPos, bool isTagged);

    virtual ~TagState() = default;
    TagState(TagState const &other);
    TagState(TagState &&) = delete;
    virtual TagState &operator=(TagState const &) = delete;
    virtual TagState &operator=(TagState &&) = delete;

    double distanceTo(State const &otherState) const;
    bool equals(State const &otherState) const;
    std::size_t hash() const;
    void print(std::ostream &os) const;

    GridPosition getRobotPosition() const {
        return robotPos;
    }

    GridPosition getOpponentPosition() const {
        return opponentPos;
    }

    bool isTagged() const {
        return isTagged_;
    }

  private:
    GridPosition robotPos, opponentPos;
    bool isTagged_;
};
#endif /* TAGSTATE_HPP */
