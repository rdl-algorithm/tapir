#ifndef ROCKSAMPLE_STATE_HPP_
#define ROCKSAMPLE_STATE_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/abstract-problem/State.hpp"             // for State
#include "solver/abstract-problem/VectorState.hpp"             // for VectorState

namespace rocksample {
class RockSampleState : public solver::VectorState {
    friend class RockSampleTextSerializer;
  public:
    RockSampleState(GridPosition position, std::vector<bool> rockStates);
    virtual ~RockSampleState() = default;

    std::unique_ptr<solver::State> copy() const override;
    double distanceTo(solver::State const &otherState) const override;
    bool equals(solver::State const &otherState) const override;
    std::size_t hash() const override;
    void print(std::ostream &os) const override;

    std::vector<double> asVector() const override;

    GridPosition getPosition() const;
    std::vector<bool> getRockStates() const;

  private:
    GridPosition position_;
    std::vector<bool> rockStates_;
};
} /* namespace rocksample */

// We define a hash function directly in the std namespace.
namespace std {
/** A struct in the std namespace to define a standard hash function for the
 * RockSampleState class.
 */
template<> struct hash<rocksample::RockSampleState> {
    /** Returns the hash value for the given TagState. */
    std::size_t operator()(rocksample::RockSampleState const &state) const {
        return state.hash();
    }
};
} /* namespace std */

#endif /* ROCKSAMPLESTATE_HPP_ */
