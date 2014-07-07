/** @file HomecareState.hpp
 *
 * Defines the HomecareState class, which represents a state of the Homecare problem.
 */
#ifndef HOMECARESTATE_HPP_
#define HOMECARESTATE_HPP_

#include <cstddef>                      // for size_t

#include <memory>
#include <ostream>                      // for ostream
#include <vector>

#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/abstract-problem/State.hpp"
#include "solver/abstract-problem/VectorState.hpp"

namespace homecare {
/** A class representing a state in the Homecare POMDP.
 *
 * The state contains the positions of the robot and the target, as well as a
 * boolean flag for whether or not the target is currently calling for assistance
 *
 * This class also implements solver::VectorState in order to allow the state
 * to be easily converted to a vector<double>, which can then be used inside 
 * the standard R*-tree implementation of StateIndex to allow spatial lookup 
 * of states.
 */
class HomecareState : public solver::VectorState {
    friend class HomecareTextSerializer;
  public:
    /** Constructs a new HomecareState with the given positions of the robot and 
     * target, and the given call state.
     */
    HomecareState(GridPosition robotPos, GridPosition targetPos, bool call);

    virtual ~HomecareState() = default;
    /** A copy constructor, for convenience. */
    HomecareState(HomecareState const &);
    /** The move constructor for HomecareState is forbidden. */
    HomecareState(HomecareState &&) = delete;
    /** The copy assignment operator for HomecareState is forbidden. */
    virtual HomecareState &operator=(HomecareState const &) = delete;
    /** The move assignment operator for HomecareState is forbidden. */
    virtual HomecareState &operator=(HomecareState &&) = delete;

    std::unique_ptr<solver::Point> copy() const override;

    double distanceTo(solver::State const &otherState) const override;
    bool equals(solver::State const &otherState) const override;
    std::size_t hash() const;

    std::vector<double> asVector() const override;
    void print(std::ostream &os) const override;

    /** Returns the position of the robot. */
    GridPosition getRobotPos() const;
    /** Returns the position of the target. */
    GridPosition getTargetPos() const;
    /** Returns true iff target is calling for help */
    bool getCall() const;
  private:
    /** The position of the robot in the grid. */
    GridPosition robotPos_;
    /** The position of the target in the grid. */
    GridPosition targetPos_;
    /** True iff target is calling for help */
    bool call_;
};
} /* namespace homecare */

// We define a hash function directly in the std namespace.
namespace std {
/** A struct in the std namespace to define a standard hash function for the HomecareState class. */
template<> struct hash<homecare::HomecareState> {
    /** Returns the hash value for the given HomecareState. */
    std::size_t operator()(homecare::HomecareState const &state) const {
        return state.hash();
    }
};
} /* namespace std */

#endif /* HOMECARESTATE_HPP_ */
