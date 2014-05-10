cmake_minimum_required(VERSION 2.8.3)
project(abt)

# Set make arguments
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wpedantic -Wall -Wextra -Wshadow -Weffc++ -Wswitch-default -Wfatal-errors -frounding-math -L/usr/lib" ) 

## Use Boost 1.48.0 from source, different to my system's default version
set(BOOST_ROOT "~/boost_1_48_0")
find_package(Boost 1.48.0 REQUIRED COMPONENTS program_options)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp roslib std_msgs geometry_msgs pcl pcl_ros tf message_generation)

add_service_files(
  FILES
  simRosCopyPasteObjects.srv
  simRosGetObjectHandle.srv
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
	INCLUDE_DIRS src
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp roslib std_msgs geometry_msgs pcl pcl_ros tf)

include_directories(src ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(AbtSolver
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
	src/solver/abstract-problem/VectorLP.cpp
	src/solver/abstract-problem/Vector.cpp
	src/solver/abstract-problem/Model.cpp
	src/solver/abstract-problem/DiscretizedPoint.cpp
	src/solver/backpropagation/AveragePropagator.cpp
	src/solver/backpropagation/BackpropagationStrategy.cpp
	src/solver/backpropagation/MaximumPropagator.cpp
	src/solver/backpropagation/RobustPropagator.cpp
	src/solver/changes/DefaultHistoryCorrector.cpp
	src/solver/indexing/FlaggingVisitor.cpp
	src/solver/indexing/RTree.cpp
	src/solver/indexing/SpatialIndexVisitor.cpp
	src/solver/mappings/approximate_observations.cpp
	src/solver/mappings/discrete_observations.cpp
	src/solver/mappings/discretized_actions.cpp
	src/solver/mappings/enumerated_actions.cpp
	src/solver/mappings/enumerated_observations.cpp
	src/solver/search/DefaultRolloutStrategy.cpp
	src/solver/search/MultipleStrategiesExp3.cpp
	src/solver/search/NnRolloutStrategy.cpp
	##src/solver/search/RandomRolloutStrategy.cpp
	src/solver/search/SearchStrategy.cpp
	src/solver/search/UcbSelectionStrategy.cpp
	src/solver/serialization/TextSerializer.cpp
	src/solver/serialization/VectorLPTextSerializer.cpp
	src/problems/shared/parsers.cpp
	src/problems/shared/geometry/Point2D.cpp
	src/problems/shared/geometry/Rectangle2D.cpp
	src/problems/shared/geometry/RTree.cpp
	src/problems/shared/geometry/utilities.cpp
	src/problems/shared/geometry/Vector2D.cpp
)

##add_library(AbtTracker
##	src/problems/tracker/TrackerAction.cpp
##	src/problems/tracker/TrackerModel.cpp
##	src/problems/tracker/TrackerObservation.cpp
##	src/problems/tracker/TrackerState.cpp
##	src/problems/tracker/TrackerTextSerializer.cpp
##)

add_library(AbtTag
	src/problems/tag/TagAction.cpp
	src/problems/tag/TagModel.cpp
	src/problems/tag/TagObservation.cpp
	src/problems/tag/TagState.cpp
	src/problems/tag/TagTextSerializer.cpp
)

target_link_libraries(AbtSolver spatialindex ${Boost_LIBRARIES} ${catkin_LIBRARIES})

##add_executable(abt_node src/abt_node.cpp)
##target_link_libraries(abt_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} AbtTracker AbtSolver )

add_executable(tag_node src/problems/tag/TagNode.cpp)
target_link_libraries(tag_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} AbtTag AbtSolver )
