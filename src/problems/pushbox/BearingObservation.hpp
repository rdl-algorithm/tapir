#pragma once

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "global.hpp"                     // for RandomGenerator

#include "solver/abstract-problem/DiscretizedPoint.hpp"
#include "solver/abstract-problem/Observation.hpp"


namespace pushbox {


/** A class representing an observation of a bearing sensor
 *
 */
class BearingObservationDiscrete: public solver::Point {
	typedef BearingObservationDiscrete This;
  public:

	BearingObservationDiscrete(int theBearing, int numberOfBuckets, const bool wasPushed): bearing(theBearing), buckets(numberOfBuckets), pushed(wasPushed) {};

    _NO_COPY_OR_MOVE(BearingObservationDiscrete);

    virtual std::unique_ptr<Point> copy() const {
    	return std::make_unique<This>(bearing, buckets, pushed);
    }

    double distanceTo(solver::Observation const &otherObs) const override {
    	This const &other =  static_cast<This const &>(otherObs);
    	return ((bearing-other.bearing) % buckets) + ( pushed == other.pushed ? 0 : buckets);
    }

    bool equals(solver::Observation const &otherObs) const override {
    	This const &other =  static_cast<This const &>(otherObs);
    	return (bearing == other.bearing) && (pushed == other.pushed);
    }

    std::size_t hash() const override {
    	return std::hash<decltype(bearing)>()(bearing);
    }

    void print(std::ostream &os) const override {
        os << bearing << (pushed ? " pushed" : " normal");
    }

    /** Returns the bearing. */
    int getBearing() const { return bearing; }

    /** Returns the number of buckets. */
    int getBuckets() const { return buckets; }

    /** returns the pushed status */
    bool getPushed() const { return pushed; }

  private:
    /** The position the robot sees itself in. */
    int bearing;
    int buckets;
    bool pushed;
};



} /* namespace pushbox */


