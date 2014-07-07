/** @file TagState.hpp
 *
 * Defines the TagState class, which represents a state of the Tag problem.
 */
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
/** A class representing a state in the Tag POMDP.
 *
 * The state contains the positions of the robot and the opponent, as well as a boolean flag for
 * whether or not the opponent has been tagged; tagged => terminal state.
 *
 * This class also implements solver::VectorState in order to allow the state to be easily
 * converted to a vector<double>, which can then be used inside the standard R*-tree implementation
 * of StateIndex to allow spatial lookup of states.
 */
class TagState : public solver::VectorState {
    friend class TagTextSerializer;
  public:
    /** Constructs a new TagState with the given positions of the robot and opponent, and the
     * given tagged state.
     */
    TagState(GridPosition robotPos, GridPosition opponentPos, bool _isTagged);

    virtual ~TagState() = default;
    /** A copy constructor, for convenience. */
    TagState(TagState const &);
    /** The move constructor for TagState is forbidden. */
    TagState(TagState &&) = delete;
    /** The copy assignment operator for TagState is forbidden. */
    virtual TagState &operator=(TagState const &) = delete;
    /** The move assignment operator for TagState is forbidden. */
    virtual TagState &operator=(TagState &&) = delete;

    std::unique_ptr<solver::Point> copy() const override;

    double distanceTo(solver::State const &otherState) const override;
    bool equals(solver::State const &otherState) const override;
    std::size_t hash() const;

    std::vector<double> asVector() const override;
    void print(std::ostream &os) const override;

    /** Returns the position of the robot. */
    GridPosition getRobotPosition() const;
    /** Returns the position of the opponent. */
    GridPosition getOpponentPosition() const;
    /** Returns true iff the opponent has already been tagged. */
    bool isTagged() const;
  private:
    /** The position of the robot in the grid. */
    GridPosition robotPos_;
    /** The position of the opponent in the grid. */
    GridPosition opponentPos_;
    /** A flag that is true iff the opponent has been tagged. */
    bool isTagged_;
};
} /* namespace tag */

// We define a hash function directly in the std namespace.
namespace std {
/** A struct in the std namespace to define a standard hash function for the TagState class. */
template<> struct hash<tag::TagState> {
    /** Returns the hash value for the given TagState. */
    std::size_t operator()(tag::TagState const &state) const {
        return state.hash();
    }
};
} /* namespace std */

#endif /* TAGSTATE_HPP_ */
