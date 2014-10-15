#pragma once


#include "solver/abstract-problem/Action.hpp"
#include "solver/mappings/actions/continuous_actions.hpp"


namespace pushbox {

/** An implementation of the 2d construction data
 *
 * This is pure continuous, so there are no extra fields necessary apart from what is in the data vector.
 */
class Action2dConstructionData final: public solver::ContinuousActionConstructionDataBase {

public:
	Action2dConstructionData() = default;

	Action2dConstructionData(const double* constructionDataVector) {
		for (size_t i=0; i< storage.size(); i++) {
			storage[i] = *(constructionDataVector+i);
		}
	}

	virtual const double* data() const override { return storage.data(); }

	double& operator[](size_t index) { return storage[index]; }
	const double& operator[](size_t index) const { return storage[index]; }

private:
	std::array<double, 2> storage = {{0,0}};
};




/** A class representing an action for 2d navigation (e.g. push box)
 *
 */
class Action2d final: public solver::ContinuousAction {
	typedef Action2d This;
  public:
    /** Constructs a new action from the given ActionType. */
	Action2d(const double x, const double y): storage() {
		storage[0] = x;
		storage[1] = y;
	};

	Action2d(const Action2dConstructionData& data):storage(data) {};

    virtual ~Action2d() = default;
    _NO_COPY_OR_MOVE(Action2d);

    std::unique_ptr<solver::Action> copy() const override {
    	return std::make_unique<This>(storage);
    }

    /** Returns true iff this point is equal to the the given point. */
    virtual bool equals(Point const &otherPoint) const override {
    	const This& other = static_cast<const This&>(otherPoint);
    	return (getX() == other.getX()) && (getY() == other.getY());
    }


    /** Returns a hash value for this point - should be consistent with equals() */
    virtual std::size_t hash() const override {
    	size_t result = 0;
    	tapir::hash_combine(result, getX());
    	tapir::hash_combine(result, getY());
    	return result;
    }

    /** returns the distance to another action */
    double distanceTo(solver::Action const &otherAction) const override {
    	struct Local {
    		static double square(const double a) { return a*a; }
    	};

    	This const &other = static_cast<This const &>(otherAction);

    	return sqrt(Local::square(getX()-other.getX())+Local::square(getY()-other.getY()));
    }

    /** print a human readable form of the action */
    void print(std::ostream &os) const override { os << "[" << getX() << " " << getY() << "]"; }


    /** Returns the content of this action. */
    double getX() const { return storage[0]; }
    double getY() const { return storage[1]; }

    virtual const solver::ContinuousActionConstructionDataBase& getConstructionData() const override {
    	return storage;
    }

  private:
    /** The actual data contained in this action.
     *
     * We use a construction data object. This makes it easy to go back and forth between construction data and the real action
     */
    Action2dConstructionData storage;
};



} /* namespace pushbox */

