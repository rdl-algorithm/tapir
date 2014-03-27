#ifndef ROCKSAMPLE_MDPSOLVER_HPP_
#define ROCKSAMPLE_MDPSOLVER_HPP_

#include <iostream>
#include <map>
#include <unordered_set>
#include <utility>

#include "global.hpp"

namespace rocksample {
class RockSampleModel;
class RockSampleState;

class RockSampleMdpSolver {
public:
    RockSampleMdpSolver(RockSampleModel *model);
    virtual ~RockSampleMdpSolver() = default;
    _NO_COPY_OR_MOVE(RockSampleMdpSolver);

    void solve();
    void getHeuristicValue(RockSampleState const &state);

    void save(std::ostream &os);
    void load(std::istream &is);
private:
    RockSampleModel *model_;
    std::map<std::pair<int, int>, double> valueMap_;
    // std::map<int, std::unordered_set<int>> rockMap_;
};

} /* namespace rocksample */

#endif /* ROCKSAMPLE_MDPSOLVER_HPP_ */
