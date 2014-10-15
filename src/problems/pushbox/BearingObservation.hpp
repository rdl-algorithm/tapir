#pragma once

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "global.hpp"                     // for RandomGenerator

#include "solver/abstract-problem/DiscretizedPoint.hpp"
#include "solver/abstract-problem/Observation.hpp"


namespace pushbox {
class ContNavModel;

/** A class representing an observation of a bearing sensor
 *
 */
class BearingObservationDiscrete: public solver::Point {
	typedef BearingObservationDiscrete This;
  public:

	BearingObservationDiscrete(int theBearing, int numberOfBuckets): bearing(theBearing), buckets(numberOfBuckets) {};

    _NO_COPY_OR_MOVE(BearingObservationDiscrete);


    double distanceTo(solver::Observation const &otherObs) const override {
    	This const &other =  static_cast<This const &>(otherObs);
    	return (bearing-other.bearing) % buckets;
    }

    bool equals(solver::Observation const &otherObs) const override {
    	This const &other =  static_cast<This const &>(otherObs);
    	return bearing == other.bearing;
    }

    std::size_t hash() const override {
    	return std::hash<decltype(bearing)>()(bearing);
    }

    void print(std::ostream &os) const override {
        os << bearing;
    }

    /** Returns the bearing. */
    const int& getBearing() const { return bearing; }


  private:
    /** The position the robot sees itself in. */
    int bearing;
    int buckets;
};



} /* namespace pushbox */


