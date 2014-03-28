#ifndef ROCKSAMPLE_MDPSOLVER_HPP_
#define ROCKSAMPLE_MDPSOLVER_HPP_

#include <iostream>
#include <map>
#include <unordered_set>
#include <utility>

#include "problems/shared/GridPosition.hpp"

#include "global.hpp"

namespace rocksample {
class RockSampleModel;
class RockSampleState;

/** Solves an MDP version of RockSample, in which it is known which rocks
 * are good and which ones are bad.
 */
class RockSampleMdpSolver {
public:
    RockSampleMdpSolver(RockSampleModel *model);
    virtual ~RockSampleMdpSolver() = default;
    _NO_COPY_OR_MOVE(RockSampleMdpSolver);

    /** Solves the MDP. */
    void solve();
    /** Calculates the exact value for the given state. */
    double getQValue(RockSampleState const &state) const;

    /** Saves the solution to a file (not implemented!) */
    void save(std::ostream &/*os*/) {}
    /** Loads the solution from a file (not implemented!) */
    void load(std::istream &/*is*/) {}
private:
    /** Calculates the q-value for the given action, from the current position
     * with the given rock state code (encoded into a number).
     *
     * The action is a simplified MDP action:
     * -1 => exit the map
     * 0+ => go to the given rock and sample it (the rock is assumed to be good)
     */
    double calculateQValue(GridPosition pos,
            long rockStateCode, long action) const;

    RockSampleModel *model_;
    std::map<std::pair<int, int>, double> valueMap_;
};

} /* namespace rocksample */

#endif /* ROCKSAMPLE_MDPSOLVER_HPP_ */
