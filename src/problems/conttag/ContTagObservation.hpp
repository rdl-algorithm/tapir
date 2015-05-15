#pragma once

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "global.hpp"                     // for RandomGenerator

#include "solver/abstract-problem/DiscretizedPoint.hpp"
#include "solver/abstract-problem/Observation.hpp"

#include "ContTagPosition.hpp"

namespace conttag {
class ContTagModel;

/** A class representing an observation in the Tag POMDP.
 *
 * This includes an observation of the robot's own position, and a boolean flag representing
 * whether or not the robot sees the opponent (and hence is on the same grid square).
 */
class ContTagObservation final: public solver::Point {
	typedef ContTagObservation This;
  public:
	ContTagObservation(const bool theSeen): seen(theSeen) {};

    virtual ~ContTagObservation() = default;
    _NO_COPY_OR_MOVE(ContTagObservation);

    std::unique_ptr<solver::Observation> copy() const override {
        return std::make_unique<This>(seen);
    }

    double distanceTo(solver::Observation const &otherObs) const override {
    	This const &other =  static_cast<This const &>(otherObs);
    	return (seen == other.seen) ? 0 : 1;
    }

    bool equals(solver::Observation const &otherObs) const override {
    	This const &other =  static_cast<This const &>(otherObs);
    	return seen == other.seen;
    }

    std::size_t hash() const override {
    	return std::hash<bool>()(seen);
    }

    void print(std::ostream &os) const override {
        os << (seen ? "seen" : "not_seen");
    }

    /** Returns the position the robot has observed itself in. */
    bool isSeen() const { return seen; }

  private:
    /** The position the robot sees itself in. */
    bool seen;
};
} /* namespace contnav */
