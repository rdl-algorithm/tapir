#ifndef UWNAV_HPP_
#define UWNAV_HPP_

namespace uwnav {
enum class UnderwaterNavAction : long {
       EAST = 0,
       NORTH = 1,
       SOUTH = 2,
       NORTHEAST = 3,
       SOUTHEAST = 4
   };

   enum class UnderwaterNavCellType : short {
       USUAL = 0,
       GOAL = 1,
       ROCK = 2,
       OBSERVATION = 3,
       SPECIAL_REWARD = 4,
       OBSTACLE = 5
   };
} /* namespace uwnav */

#endif /* UWNAV_HPP_ */
