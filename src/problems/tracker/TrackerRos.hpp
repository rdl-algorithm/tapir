#ifndef TRACKERROS_HPP_
#define TRACKERROS_HPP_


#include "problems/shared/GridPosition.hpp"

namespace tracker {

	GridPosition getCurrCell();

	// Returns current yaw in degrees discretised by 45 degree steps
	int getCurrYaw45();

	// Returns true if ray cast from start to target is obstructed
	bool isRayBlocked(GridPosition start, GridPosition target);

} /* namespace tracker */

#endif /* TRACKERMODEL_HPP_ */