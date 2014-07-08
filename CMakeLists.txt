cmake_minimum_required(VERSION 2.8.3)
project(tapir)

# Set make arguments
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -frounding-math" ) 

## Use Boost 1.48.0 from source, different to my system's default version
##set(BOOST_ROOT "~/boost_1_48_0")
##find_package(Boost 1.48.0 REQUIRED)

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
  simRosGetObjectPose.srv
  simRosLoadScene.srv
  simRosSetObjectPosition.srv
  simRosStartSimulation.srv
  simRosStopSimulation.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package(
	INCLUDE_DIRS src src/options src/problems/shared/ros
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp roslib std_msgs geometry_msgs tf message_runtime laser_geometry)

include_directories(src src/options src/problems/shared/ros ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(TapirSolver
	src/global.cpp
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
	src/solver/abstract-problem/heuristics/RolloutHeuristic.cpp
	src/solver/belief-estimators/estimators.cpp
	src/solver/changes/DefaultHistoryCorrector.cpp
	src/solver/indexing/FlaggingVisitor.cpp
	src/solver/indexing/RTree.cpp
	src/solver/indexing/SpatialIndexVisitor.cpp
	src/solver/mappings/actions/discretized_actions.cpp
	src/solver/mappings/actions/enumerated_actions.cpp
	src/solver/mappings/observations/approximate_observations.cpp
	src/solver/mappings/observations/discrete_observations.cpp
	src/solver/mappings/observations/enumerated_observations.cpp
	src/solver/search/MultipleStrategiesExp3.cpp
	src/solver/search/search_interface.cpp
	src/solver/search/action-choosers/choosers.cpp
	src/solver/search/steppers/default_rollout.cpp
	src/solver/search/steppers/nn_rollout.cpp
	src/solver/search/steppers/ucb_search.cpp
	src/solver/serialization/TextSerializer.cpp
	src/options/inih/ini.c
	src/options/option_parser.cpp
	src/problems/shared/parsers.cpp
	src/problems/shared/geometry/Point2D.cpp
	src/problems/shared/geometry/Rectangle2D.cpp
	src/problems/shared/geometry/RTree.cpp
	src/problems/shared/geometry/utilities.cpp
	src/problems/shared/geometry/Vector2D.cpp
	src/problems/shared/policy_iteration.cpp
)

add_library(TapirRos
	src/problems/shared/ros/VrepHelper.cpp
)

add_dependencies(TapirRos tapir_generate_messages_cpp)

add_library(TapirTracker
	src/problems/tracker/TrackerAction.cpp
	src/problems/tracker/TrackerModel.cpp
	src/problems/tracker/TrackerObservation.cpp
	src/problems/tracker/TrackerState.cpp
	src/problems/tracker/TrackerTextSerializer.cpp
)

add_library(TapirTag
	src/problems/tag/TagAction.cpp
	src/problems/tag/TagMdpSolver.cpp
	src/problems/tag/TagModel.cpp
	src/problems/tag/TagObservation.cpp
	src/problems/tag/TagState.cpp
	src/problems/tag/TagTextSerializer.cpp
)

target_link_libraries(TapirSolver spatialindex ${catkin_LIBRARIES})

add_executable(tracker_node src/problems/tracker/TrackerRos.cpp)
target_link_libraries(tracker_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} TapirTracker TapirSolver TapirRos)
add_dependencies(tracker_node tapir_generate_messages_cpp)

add_executable(tag_node src/problems/tag/ros/TagVrep.cpp)
target_link_libraries(tag_node ${catkin_LIBRARIES} TapirTag TapirSolver TapirRos)
add_dependencies(tag_node tapir_generate_messages_cpp)

add_executable(scan_to_pcl_node src/problems/shared/ros/ScanToPcl.cpp)
target_link_libraries(scan_to_pcl_node ${catkin_LIBRARIES})
