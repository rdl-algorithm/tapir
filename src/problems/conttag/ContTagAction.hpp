#pragma once

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/DiscretizedPoint.hpp"             // for DiscretizedPoint
#include "solver/mappings/actions/continuous_actions.hpp"
#include <boost/math/constants/constants.hpp>

namespace conttag {


/** An implementation of the 2d construction data
 *
 * This is pure continuous, so there are no extra fields necessary apart from what is in the data vector.
 */
class ContTagActionConstructionData final: public solver::ContinuousActionConstructionDataBase {
	typedef ContTagActionConstructionData This;
	typedef solver::ContinuousActionConstructionDataBase Base;
public:

	static double constexpr tagAction = 1000;

	ContTagActionConstructionData() = default;

	ContTagActionConstructionData(const double* constructionDataVector) {
		for (size_t i=0; i< storage.size(); i++) {
			storage[i] = *(constructionDataVector+i);
		}
	}

	ContTagActionConstructionData(const double alpha): storage({alpha}) {};


	virtual const double* data() const override { return storage.data(); }

	size_t size() const { return storage.size(); }
	double& operator[](size_t index) { return storage[index]; }
	const double& operator[](size_t index) const { return storage[index]; }

public:
	/* Infrastructure for use in a ContinuousActionContainer */
	struct HashEqualOptions{
		HashEqualOptions(const double theTolerance): tolerance(theTolerance) {}

		/* Defines the tolerance used when hashing and comparing elements. */
		const double tolerance;
	};

	size_t hash(const HashEqualOptions& options) const {
		size_t result = 0;
		for (auto i : storage) {
			tapir::hash_combine(result, snapToTolerance(i, options.tolerance));
		}
		return result;
	}

	bool equal(const This& other, const HashEqualOptions& options) const {
		for (size_t i = 0; i<storage.size(); i++) {
			if ( snapToTolerance(storage[i], options.tolerance) != snapToTolerance(other.storage[i], options.tolerance) ) {
				return false;
			}
		}
		return true;
	}

	virtual bool equal(const Base& /*other*/) const override {
		TAPIR_THROW("This has not been implemented yet.");
		return false;
	}

private:
	static double snapToTolerance(const double value, const double tolerance) { return std::round(value/tolerance)*tolerance; }


private:
	std::array<double, 1> storage = {{0}};
};




/** A class representing an action for 2d navigation (e.g. push box)
 *
 */
class ContTagAction final: public solver::ContinuousAction {
	typedef ContTagAction This;
  public:
	typedef ContTagActionConstructionData ConstructionData;

	constexpr static double const tagAction = ConstructionData::tagAction;

    /** Constructs a new action from the given ActionType. */
	ContTagAction(const double x, const double y): storage() {
		storage[0] = x;
		storage[1] = y;
	};

	ContTagAction(const ConstructionData& data):storage(data) {};

	ContTagAction(const double* constructionDataVector):storage(constructionDataVector) {};

    virtual ~ContTagAction() = default;
    _NO_COPY_OR_MOVE(ContTagAction);

    std::unique_ptr<solver::Action> copy() const override {
    	auto result = std::make_unique<This>(storage);
    	result->binNumber = binNumber;
    	return std::move(result);
    }

    /** Returns true iff this point is equal to the the given point. */
    virtual bool equals(Point const &otherPoint) const override {
    	const This& other = static_cast<const This&>(otherPoint);
    	return (getAngle() == other.getAngle());
    }


    /** Returns a hash value for this point - should be consistent with equals() */
    virtual std::size_t hash() const override {
    	return std::hash<double>()(getAngle());
    }

    /** returns the distance to another action */
    double distanceTo(solver::Action const &otherAction) const override {

    	This const &other = static_cast<This const &>(otherAction);

    	if (isTag()) {
    		if (other.isTag()) {
    			return 0;
    		} else {
    			return 1000;
    		}
    	} else {
    		return std::fabs(getAngle()-other.getAngle());
    	}
    }

    /** print a human readable form of the action */
    void print(std::ostream &os) const override {
    	if (isTag()) {
    		os << "TAG";
    	} else {
    		os << getAngle();
    	}
    }

    virtual std::vector<double> getVectorForm() const override {
        	return {getAngle()};
    };

    /** Returns the content of this action. */
    double getAngle() const { return storage[0]; }
    bool isTag() const { return getAngle() == tagAction; }

    virtual const solver::ContinuousActionConstructionDataBase& getConstructionData() const override {
    	return storage;
    }


    virtual long getBinNumber() const override {
    	if (binNumber < 0) {
    		TAPIR_THROW("The bin number has not been assigned.");
    	}
    	return binNumber;
    }

    void setBinNumber(const long theBinNumber) { binNumber = theBinNumber; }

  private:
    /** The actual data contained in this action.
     *
     * We use a construction data object. This makes it easy to go back and forth between construction data and the real action
     */
    ConstructionData storage;

    long binNumber = -1;
};



} /* namespace conttag */

