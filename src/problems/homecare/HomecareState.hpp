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
class HomecareState : public solver::VectorState {
    friend class HomecareTextSerializer;
  public:
    HomecareState(GridPosition robotPos, GridPosition targetPos, bool call);

    virtual ~HomecareState() = default;
    HomecareState(HomecareState const &);
    HomecareState(HomecareState &&) = delete;
    virtual HomecareState &operator=(HomecareState const &) = delete;
    virtual HomecareState &operator=(HomecareState &&) = delete;

    std::unique_ptr<solver::Point> copy() const override;

    double distanceTo(solver::State const &otherState) const override;
    bool equals(solver::State const &otherState) const override;
    std::size_t hash() const;

    std::vector<double> asVector() const override;
    void print(std::ostream &os) const override;

    GridPosition getRobotPos() const;
    GridPosition getTargetPos() const;
    bool getCall() const;
  private:
    GridPosition robotPos_, targetPos_;
    bool call_;
};
} /* namespace homecare */

// We define a hash function directly in the std namespace.
namespace std {
template<> struct hash<homecare::HomecareState> {
    std::size_t operator()(homecare::HomecareState const &state) const {
        return state.hash();
    }
};
} /* namespace std */

#endif /* HOMECARESTATE_HPP_ */
