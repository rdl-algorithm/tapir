cmake_minimum_required(VERSION 2.8.3)
project(abt)

# Set make arguments
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wpedantic -Wall -Wextra -Wshadow -Weffc++ -Wswitch-default -Wfatal-errors -frounding-math -L/usr/lib" ) 

## Use Boost 1.48.0 from source, different to my system's default version
set(BOOST_ROOT "~/boost_1_48_0")
find_package(Boost 1.48.0 REQUIRED COMPONENTS program_options)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp roslib std_msgs geometry_msgs tf message_generation laser_geometry)

## Find OctoMap package
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS} )
link_libraries(${OCTOMAP_LIBRARIES})
link_directories(${OCTOMAP_LIBRARY_DIRS})

## Find PCL package
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

add_message_files(
  FILES
  VrepInfo.msg
)

add_service_files(
  FILES
  simRosCopyPasteObjects.srv
  simRosGetObjectHandle.srv
  simRosSetObjectPosition.srv
  simRosStartSimulation.srv
  simRosStopSimulation.srv
  simRosGetObjectPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package(
	INCLUDE_DIRS src
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp roslib std_msgs geometry_msgs tf message_runtime laser_geometry)

include_directories(src ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(AbtSolver
	src/global.cpp
	src/VrepHelper.cpp
	src/solver/ActionNode.cpp
	src/solver/Agent.cpp
	src/solver/BeliefNode.cpp
	src/solver/BeliefTree.cpp
	src/solver/Histories.cpp
	src/solver/HistoryEntry.cpp
	src/solver/HistorySequence.cpp
	src/solver/Simulator.cpp
	src/solver/Solver.cpp
	src/solver/StateInfo.cpp
	src/solver/StatePool.cpp
	src/solver/abstract-problem/DiscretizedPoint.cpp
	src/solver/abstract-problem/Model.cpp
	src/solver/abstract-problem/Vector.cpp
	src/solver/abstract-problem/heuristics/rollout_heuristic.cpp
	src/solver/belief-estimators/estimators.cpp
	src/solver/changes/DefaultHistoryCorrector.cpp
	src/solver/indexing/FlaggingVisitor.cpp
	src/solver/indexing/RTree.cpp
	src/solver/indexing/SpatialIndexVisitor.cpp
	src/solver/mappings/actions/ActionMappingEntry.cpp
	src/solver/mappings/actions/discretized_actions.cpp
	src/solver/mappings/actions/enumerated_actions.cpp
	src/solver/mappings/observations/approximate_observations.cpp
	src/solver/mappings/observations/discrete_observations.cpp
	src/solver/mappings/observations/enumerated_observations.cpp
	src/solver/search/exp3.cpp
	src/solver/search/search_interface.cpp
	src/solver/search/action-choosers/choosers.cpp
	src/solver/search/steppers/default_rollout.cpp
	src/solver/search/steppers/nn_rollout.cpp
	src/solver/search/steppers/ucb_search.cpp
	src/solver/serialization/TextSerializer.cpp
	src/problems/shared/parsers.cpp
	src/problems/shared/geometry/Point2D.cpp
	src/problems/shared/geometry/Rectangle2D.cpp
	src/problems/shared/geometry/RTree.cpp
	src/problems/shared/geometry/utilities.cpp
	src/problems/shared/geometry/Vector2D.cpp
)

add_library(AbtTracker
	src/problems/tracker/TrackerAction.cpp
	src/problems/tracker/TrackerModel.cpp
	src/problems/tracker/TrackerObservation.cpp
	src/problems/tracker/TrackerState.cpp
	src/problems/tracker/TrackerTextSerializer.cpp
)

add_library(AbtTag
	src/problems/tag/TagAction.cpp
	src/problems/tag/TagModel.cpp
	src/problems/tag/TagObservation.cpp
	src/problems/tag/TagState.cpp
	src/problems/tag/TagTextSerializer.cpp
)

target_link_libraries(AbtSolver spatialindex ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(tracker_node src/problems/tracker/TrackerRos.cpp)
target_link_libraries(tracker_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} AbtTracker AbtSolver )

add_executable(tag_node src/problems/tag/TagVrep.cpp)
target_link_libraries(tag_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} AbtTag AbtSolver )

add_executable(scan_to_pcl_node src/ScanToPcl.cpp)
target_link_libraries(scan_to_pcl_node ${Boost_LIBRARIES} ${catkin_LIBRARIES})
