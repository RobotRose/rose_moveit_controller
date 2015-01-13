cmake_minimum_required(VERSION 2.8.3)
project(rose_moveit_controller)
find_package(catkin 
	REQUIRED COMPONENTS
		actionlib 
		actionlib_msgs
		genmsg
		geometry_msgs
		# moveit_core 
		# moveit_ros_planning 
		moveit_ros_planning_interface 
		pluginlib 
		roscpp
		rospy 
		sensor_msgs 
		std_msgs 
)

add_action_files(
	DIRECTORY 
		action 
	FILES 
		arm_goal.action)

generate_messages( DEPENDENCIES 
	std_msgs
	geometry_msgs
	actionlib_msgs
	)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS
		actionlib_msgs
		geometry_msgs
		interactive_markers
		message_generation
		message_runtime
		# moveit_core
		moveit_ros_planning_interface
		roscpp 
		sensor_msgs
		std_msgs 
	DEPENDS roscpp Boost Eigen
)

set(CMAKE_CXX_FLAGS "-std=c++0x -g -lcurl") # TODO OH: -g is DEBUG FLAG 
											# TODO MDL: -lcurl is not so nice (but leaving it out caused linking error)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN_INCLUDE_DIRS}
)

add_executable(rose_moveit_controller src/moveit_controller.cpp)

add_dependencies(rose_moveit_controller ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(rose_moveit_controller
  ${catkin_LIBRARIES}
)

link_directories(${catkin_LIBRARY_DIRS})