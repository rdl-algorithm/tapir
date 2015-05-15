#pragma once

#include <cstddef>                      // for size_t

#include <memory>
#include <ostream>                      // for ostream
#include <vector>
#include <functional>

#include "solver/abstract-problem/State.hpp"
#include "solver/abstract-problem/VectorState.hpp"
#include "ContTagPosition.hpp"

namespace conttag {
/** A class representing a state in the Tag POMDP.
 *
 * The state contains the positions of the robot and the opponent, as well as a boolean flag for
 * whether or not the opponent has been tagged; tagged => terminal state.
 *
 * This class also implements solver::VectorState in order to allow the state to be easily
 * converted to a vector<double>, which can then be used inside the standard R*-tree implementation
 * of StateIndex to allow spatial lookup of states.
 */
class ContTagState final: public solver::VectorState {
	typedef ContTagState This;
  public:
	typedef ContTagPosition Position;
	typedef ContTagPosition::real real;

    /** Constructs a new TagState with the given positions of the robot and opponent, and the
     * given tagged state.
     */
	ContTagState(const Position& therobotPos, const double theRobotAngle, const Position& theHumanPos, const bool theHumanIsTagged): robotPos(therobotPos), robotAngle(theRobotAngle), humanPos(theHumanPos), humanIsTagged(theHumanIsTagged) {};

    virtual ~ContTagState() = default;
    /** A copy constructor, for convenience. */
    ContTagState(ContTagState const &) = default;
    /** The move constructor for TagState is forbidden. */
    ContTagState(ContTagState &&) = delete;
    /** The copy assignment operator for TagState is forbidden. */
    virtual ContTagState &operator=(ContTagState const &) = delete;
    /** The move assignment operator for TagState is forbidden. */
    virtual ContTagState &operator=(ContTagState &&) = delete;

    std::unique_ptr<solver::Point> copy() const override {
    	return std::make_unique<ContTagState>(robotPos, robotAngle, humanPos, humanIsTagged);
    }

    double distanceTo(solver::State const &otherState) const override {
    	This const &other = static_cast<This const &>(otherState);
    	if (humanIsTagged != other.humanIsTagged) {
    		return 1000;
    	}

        real sum = square(robotPos.x-other.robotPos.x) + square(robotPos.y-other.robotPos.y);
        sum += square(humanPos.x-other.humanPos.x) + square(humanPos.y-other.humanPos.y);
        sum += square(centerAngle(robotAngle - other.robotAngle));
        return sqrt(sum);
    }

    bool equals(solver::State const &otherState) const override {
    	This const &other = static_cast<This const &>(otherState);
    	return (robotPos == other.robotPos) && (humanPos == other.humanPos) && (robotAngle == other.robotAngle) && (humanIsTagged == other.humanIsTagged);
    }

    std::size_t hash() const {
    	size_t result = 0;
    	tapir::hash_combine(result, robotPos);
    	tapir::hash_combine(result, robotAngle);
    	tapir::hash_combine(result, humanPos);
    	tapir::hash_combine(result, humanIsTagged);
    	return result;
    }

    std::vector<double> asVector() const override{
        return {robotPos.x, robotPos.y, robotAngle, humanPos.x, humanPos.y, humanIsTagged ? double(1) : 0};
    }

    void print(std::ostream &os) const override {
        os << "ROBOT: " << robotPos << " ANGLE: " << robotAngle << " HUMAN: " << humanPos;
        os << " " << (humanIsTagged ? "TAGGED" : "not_tagged");
    }

    /** Returns the position of the robot. */
    const Position& getRobotPosition() const { return robotPos; }
    const Position& getHumanPosition() const { return humanPos; }
    double getRobotAngle() const { return robotAngle; }
    bool isTagged() const { return humanIsTagged; }

  public:
    static real square(const real x) { return x*x; }

    static real centerAngle(const real alpha){
        if (alpha>M_PI){
            return fmod(alpha+M_PI, 2*M_PI)-M_PI;
        }else if(alpha< -M_PI){
            return -(fmod(-alpha+M_PI, 2*M_PI)-M_PI);
        }
        return alpha;
    }


  private:
    /** The position of the robot in the grid. */
    Position robotPos;
    double robotAngle;
    Position humanPos;
    bool humanIsTagged;
};
} /* namespace contnav */

// We define a hash function directly in the std namespace.
namespace std {
/** A struct in the std namespace to define a standard hash function for the ContTagState class. */
template<> struct hash<conttag::ContTagState> {
    /** Returns the hash value for the given TagState. */
    std::size_t operator()(conttag::ContTagState const &state) const {
        return state.hash();
    }
};
} /* namespace std */

