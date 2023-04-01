execute_process(COMMAND bash -c "env | grep ROS" OUTPUT_VARIABLE ENV_VAL)
message(STATUS "Env variables:\n${ENV_VAL}")
if ("$ENV{ROS_VERSION}" STREQUAL "2")
	cmake_minimum_required(VERSION 3.8)
	set(PACKAGE_ROS_VERSION 2)
	set(ROS_2 TRUE)
else()
	cmake_minimum_required(VERSION 3.1)
	set(PACKAGE_ROS_VERSION 1)
	set(ROS_1 TRUE)
endif()

project(pose_cov_ops)

# -----------------------------------------
# ROS 1
# -----------------------------------------
if (ROS_1)
	## Find catkin macros and libraries
	## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
	## is used, also find other catkin packages
	find_package(catkin REQUIRED COMPONENTS
		geometry_msgs
		tf2
		roscpp
	)

	###################################
	## catkin specific configuration ##
	###################################
	## The catkin_package macro generates cmake config files for your package
	## Declare things to be passed to dependent projects
	## INCLUDE_DIRS: uncomment this if you package contains header files
	## LIBRARIES: libraries you create in this project that dependent projects also need
	## CATKIN_DEPENDS: catkin_packages dependent projects also need
	## DEPENDS: system dependencies of this project that dependent projects also need
	catkin_package(
		INCLUDE_DIRS include
		LIBRARIES ${PROJECT_NAME}
		CATKIN_DEPENDS
				geometry_msgs
				tf2
		)
		
	find_package(mrpt-ros1bridge REQUIRED)
endif()

# -----------------------------------------
# ROS 2
# -----------------------------------------
if (ROS_2)
	# find dependencies
	find_package(ament_cmake REQUIRED)
	
	find_package(geometry_msgs REQUIRED)
	find_package(tf2 REQUIRED)
	find_package(mrpt-ros2bridge REQUIRED)
endif()

# -----------------------------------------
# ROS 1 & 2 common deps:
# -----------------------------------------
find_package(mrpt-poses REQUIRED)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
if (catkin_INCLUDE_DIRS)
	include_directories(include ${catkin_INCLUDE_DIRS})
endif()

## Declare a cpp library
add_library(${PROJECT_NAME} SHARED
   src/${PROJECT_NAME}.cpp
)

# Done automatically via mrpt exported cmake flags
# set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	# High level of warnings.
	target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

	# The -Wno-long-long is required in 64bit systems when including sytem headers.
	# The -Wno-variadic-macros was needed for Eigen3, StdVector.h
	target_compile_options(${PROJECT_NAME} PUBLIC -Wno-long-long -Wno-variadic-macros)
	# Workaround: Eigen <3.4 produces *tons* of warnings in GCC >=6. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1221
	if (NOT ${CMAKE_CXX_COMPILER_VERSION} LESS "6.0")
		target_compile_options(${PROJECT_NAME} PUBLIC -Wno-ignored-attributes -Wno-int-in-bool-context)
	endif()
endif()

## Declare a cpp executable
# add_executable(pose_cov_ops_node src/pose_cov_ops_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(pose_cov_ops_node pose_cov_ops_generate_messages_cpp)

if (ROS_1)
	target_link_libraries(${PROJECT_NAME} 
		PRIVATE
		${catkin_LIBRARIES}
		mrpt::ros1bridge
		mrpt::poses
		)
else()
	ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs tf2)

	target_link_libraries(${PROJECT_NAME} 
		mrpt::ros2bridge
		mrpt::poses
		)
endif()


target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
  PACKAGE_ROS_VERSION=${PACKAGE_ROS_VERSION}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
# in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark executables and/or libraries for installation
if (ROS_2)
	set(CATKIN_PACKAGE_LIB_DESTINATION lib)
	set(CATKIN_PACKAGE_BIN_DESTINATION bin)
	set(CATKIN_PACKAGE_INCLUDE_DESTINATION include)
endif()

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############
# Add gtest based cpp test target and link libraries
if (ROS_1)
	catkin_add_gtest(${PROJECT_NAME}-test test/test_pose_cov_ops.cpp)
endif()

if (ROS_2)
	#find_package(ament_cmake_copyright REQUIRED)
	#find_package(ament_cmake_cppcheck REQUIRED)
	#find_package(ament_cmake_cpplint REQUIRED)
	#find_package(ament_cmake_lint_cmake REQUIRED)
	#find_package(ament_cmake_uncrustify REQUIRED)
	find_package(ament_cmake_xmllint REQUIRED)

	#ament_copyright()
	#ament_cpplint()
	#ament_lint_cmake()
	ament_xmllint()

	find_package(ament_cmake_gtest REQUIRED)

	ament_add_gtest(${PROJECT_NAME}-test test/test_pose_cov_ops.cpp)

	ament_export_dependencies(mrpt-ros2bridge)

	# Export old-style CMake variables
	ament_export_include_directories("include/${PROJECT_NAME}")
	ament_export_libraries(${PROJECT_NAME})

	# Export modern CMake targets
	ament_export_targets(export_${PROJECT_NAME})

	ament_package()
endif()

if(TARGET ${PROJECT_NAME}-test)
	target_link_libraries(${PROJECT_NAME}-test
		${PROJECT_NAME}
		mrpt::poses
		)
	if (ROS_1)
		target_link_libraries(${PROJECT_NAME}-test mrpt::ros1bridge)
	else()
		target_link_libraries(${PROJECT_NAME}-test mrpt::ros2bridge)
	endif()
endif()
