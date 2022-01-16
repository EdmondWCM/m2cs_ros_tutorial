# 1 Required CMake version
cmake_minimum_required(VERSION 2.8.3)

# 2 Package name
project(m2_ps4)

# 3 Find other CMake/Catkin packages needed
find_package(catkin REQUIRED COMPONENTS rospy message_generation std_msgs geometry_msgs)

# 4 Enable Python module support
catkin_python_setup()

# 5 Message/Service/Action generators
add_message_files(
	FILES
	Ps4Data.msg
	Ps4Tpad.msg
	Ps4Motion.msg
	RgbTime.msg
	Rumble.msg
	ChannelProvider.msg
)

add_service_files(FILES
    SetRgb.srv
)

# 6 Generate message/service/action
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

# 7 Package build info export
catkin_package(
#	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS message_runtime
#	DEPENDS
)

# 7.1 Include paths and library paths
include_directories(include ${catkin_INCLUDE_DIRS})
#link_directories()

# 8 Libraries/Executables to build
# http://www.mariobadr.com/creating-a-header-only-library-with-cmake.html
# add_library(${PROJECT_NAME} INTERFACE)
# target_include_directories(${PROJECT_NAME} INTERFACE include)

# add_dependencies(ps4_to_msg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(ps4_to_msg ${catkin_LIBRARIES})
#
# ## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
# DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# FILES_MATCHING PATTERN "*.hpp"
# PATTERN ".svn" EXCLUDE
# )
#
#add_dependencies(
#	${PROJECT_NAME}
#	${${PROJECT_NAME}_EXPORTED_TARGETS}
#	${catkin_EXPORTED_TARGETS}
#)
#add_executable()
#target_link_libraries()

# 9 Tests to build
#catkin_add_gtest()

# 10. Install rules
#install()
