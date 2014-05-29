#ifndef TRACKERROS_HPP_
#define TRACKERROS_HPP_


#include "problems/shared/GridPosition.hpp"

namespace tracker {

	GridPosition getCurrCell();

	// Returns current yaw in degrees discretised by 45 degree steps
	int getCurrYaw45();

} /* namespace tracker */

#endif /* TRACKERMODEL_HPP_ */