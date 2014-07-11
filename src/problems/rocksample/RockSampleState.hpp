/** @file RockSampleState.hpp
 *
 * Defines the RockSampleState class, which represents a state of the RockSample problem.
 */
#ifndef ROCKSAMPLE_STATE_HPP_
#define ROCKSAMPLE_STATE_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/abstract-problem/State.hpp"             // for State
#include "solver/abstract-problem/VectorState.hpp"             // for VectorState

namespace rocksample {
/** A class representing a state in the RockSample POMDP.
 *
 * The state contains the position of the robot, as well as a boolean value for each rock
 * representing whether it is good (true => good, false => bad).
 *
 * This class also implements solver::VectorState in order to allow the state to be easily
 * converted to a vector<double>, which can then be used inside the standard R*-tree implementation
 * of StateIndex to allow spatial lookup of states.
 */
class RockSampleState : public solver::VectorState {
    friend class RockSampleTextSerializer;
  public:
    /** Constructs a new RockSampleState with the given robot position, and the given goodness states
     * for all of the rocks.
     */
    RockSampleState(GridPosition position, std::vector<bool> rockStates);
    virtual ~RockSampleState() = default;

    std::unique_ptr<solver::State> copy() const override;
    double distanceTo(solver::State const &otherState) const override;
    bool equals(solver::State const &otherState) const override;
    std::size_t hash() const override;
    void print(std::ostream &os) const override;

    std::vector<double> asVector() const override;

    /** Returns the position of the robot. */
    GridPosition getPosition() const;
    /** Returns the goodness states for all of the rocks (true => good). */
    std::vector<bool> getRockStates() const;

  private:
    /** The position of the robot. */
    GridPosition position_;
    /** The goodness states of the rocks in this RockSampleState. */
    std::vector<bool> rockStates_;
};
} /* namespace rocksample */

// We define a hash function directly in the std namespace.
namespace std {
/** A struct in the std namespace to define a standard hash function for the
 * RockSampleState class.
 */
template<> struct hash<rocksample::RockSampleState> {
    /** Returns the hash value for the given RockSampleState. */
    std::size_t operator()(rocksample::RockSampleState const &state) const {
        return state.hash();
    }
};
} /* namespace std */

#endif /* ROCKSAMPLESTATE_HPP_ */
