#ifndef NAV2D_ACTION_HPP_
#define NAV2D_ACTION_HPP_

#include <cstddef>                      // for size_t

#include <memory>
#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/DiscretizedPoint.hpp"

namespace nav2d {
class Nav2DModel;
class Nav2DSpcHistoryCorrector;

class Nav2DAction : public solver::DiscretizedPoint {
    friend class Nav2DTextSerializer;
    friend class Nav2DSpcHistoryCorrector;
  public:
    Nav2DAction(double speed, double rotationalSpeed, long binNumber);
    Nav2DAction(double speed, double rotationalSpeed, Nav2DModel *model);
    Nav2DAction(long binNo, Nav2DModel *model);

    virtual ~Nav2DAction() = default;
    // Copy constructor is allowed, but not others.
    Nav2DAction(Nav2DAction const &other);
    Nav2DAction(Nav2DAction &&) = delete;
    Nav2DAction &operator=(Nav2DAction const &) = delete;
    Nav2DAction &operator=(Nav2DAction &&) = delete;

    std::unique_ptr<solver::Action> copy() const override;
    double distanceTo(solver::Action const &otherAction) const override;
    bool equals(solver::Action const &otherAction) const override;
    std::size_t hash() const override;
    void print(std::ostream &os) const override;

    double getSpeed() const;
    double getRotationalSpeed() const;
    long getBinNumber() const override;

    static long getNumberOfBins();
    static double getValue(long index,
            std::vector<double> const &cutoffs);
    static long getIndex(double value,
            std::vector<double> const &cutoffs);

    static double getSpeed(long binNo, Nav2DModel *model);
    static double getRotationalSpeed(long binNo, Nav2DModel *model);
    static long calculateBinNumber(double speed,
            double rotationalSpeed, Nav2DModel *model);
private:
    static std::vector<double> SPEED_CUTOFFS;
    static std::vector<double> ROTATIONAL_SPEED_CUTOFFS;
    double speed_;
    double rotationalSpeed_;
    long binNumber_;
};
} /* namespace nav2d */

#endif /* NAV2D_ACTION_HPP_ */
