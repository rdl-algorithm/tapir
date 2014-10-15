#pragma once

#include <cstddef>                      // for size_t

#include <memory>
#include <ostream>                      // for ostream
#include <vector>
#include <functional>

#include "solver/abstract-problem/VectorState.hpp"
#include "Position2d.hpp"

namespace pushbox {

/** A class representing a state in a POMDP where we have the robot and an opponent's position. (e.g. pushbox)
 *
 * The state contains the positions of the robot and the opponent
 */
class StateRobotWithOpponent2d final: public solver::VectorState {
	typedef StateRobotWithOpponent2d This;
  public:
	typedef Position2d Position;

    /** Constructs a new TagState with the given positions of the robot and opponent, and the
     * given tagged state.
     */
	StateRobotWithOpponent2d(const Position& theRobotPos, const Position& theOpponentPos): robotPos(theRobotPos), opponentPos(theOpponentPos) {};

	StateRobotWithOpponent2d(const double x, const double y, const double opponentPosX, const double opponentPosY): robotPos(x, y), opponentPos(opponentPosX, opponentPosY) {};

    virtual ~StateRobotWithOpponent2d() = default;
    _NO_COPY_OR_MOVE(StateRobotWithOpponent2d);

//    /** A copy constructor, for convenience. */
//    StateRobotWithOpponent2d(StateRobotWithOpponent2d const &) = default;
//    /** The move constructor for TagState is forbidden. */
//    StateRobotWithOpponent2d(StateRobotWithOpponent2d &&) = delete;
//    /** The copy assignment operator is forbidden. */
//    virtual This &operator=(This const &) = delete;
//    /** The move assignment operator for TagState is forbidden. */
//    virtual This &operator=(This &&) = delete;

    std::unique_ptr<solver::Point> copy() const override {
    	return std::make_unique<This>(robotPos, opponentPos);
    }

    double distanceTo(solver::State const &otherState) const override {
    	This const &other = static_cast<This const &>(otherState);
        return std::sqrt(robotPos.euclideanDistanceSquaredTo(other.robotPos) + opponentPos.euclideanDistanceSquaredTo(other.opponentPos));
    }

    bool equals(solver::State const &otherState) const override {
    	This const &other = static_cast<This const &>(otherState);
    	return (robotPos == other.robotPos) && (opponentPos == other.opponentPos);
    }

    std::size_t hash() const {
    	size_t result = 0;
    	tapir::hash_combine(result, robotPos);
    	tapir::hash_combine(result, opponentPos);
    	return result;
    }

    std::vector<double> asVector() const override{
        return {robotPos.x, robotPos.y, opponentPos.x, opponentPos.y};
    }

    void print(std::ostream &os) const override {
        os << "ROBOT: " << robotPos << " OPPONENT: " << opponentPos;
    }

    /** Returns the position of the robot. */
    const Position& getRobotPosition() const { return robotPos; }

    /** Returns the position of the robot. */
    Position& getRobotPosition() { return robotPos; }

    /** Returns the position of the opponent. */
    const Position& getOpponentPosition() const { return opponentPos; }

    /** Returns the position of the opponent. */
    Position& getOpponentPosition() { return opponentPos; }


  private:
    /** The position of the robot. */
    Position robotPos;

    /** The position of the opponent. */
    Position opponentPos;
};


} /* namespace pushbox */



// We define a hash function directly in the std namespace.
namespace std {
/** A struct in the std namespace to define a standard hash function for the StateRobotWithOpponent2d class. */
template<> struct hash<pushbox::StateRobotWithOpponent2d> {
    /** Returns the hash value for the given TagState. */
    std::size_t operator()(pushbox::StateRobotWithOpponent2d const &state) const {
        return state.hash();
    }
};
} /* namespace std */

