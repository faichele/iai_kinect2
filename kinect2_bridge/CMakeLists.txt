cmake_minimum_required(VERSION 2.8.3)
project(kinect2_bridge CXX)

if (UNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBT_USE_DOUBLE_PRECISION -Wall")
	# Unused warnings
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self -Wunused-function -Wunused-label -Wunused-variable -Wunused-but-set-variable -Wunused-but-set-parameter")
	# Additional warnings
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Warray-bounds -Wtype-limits -Wreturn-type -Wsequence-point -Wparentheses -Wmissing-braces -Wchar-subscripts -Wswitch -Wwrite-strings -Wenum-compare -Wempty-body -Wlogical-op")

	# Check for c++11 support
	INCLUDE(CheckCXXCompilerFlag)
	CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
	CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
	IF(COMPILER_SUPPORTS_CXX11)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	ELSEIF(COMPILER_SUPPORTS_CXX0X)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
	ELSE()
	  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
	ENDIF()
endif()

# additional cmake modules
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

if (UNIX)
	find_package(freenect2 REQUIRED HINTS "$ENV{HOME}/freenect2")
elseif (WIN32)
	find_package(Kinect2SDK REQUIRED)
        find_package(Qt5Core REQUIRED)
        find_package(Qt5WinExtras REQUIRED)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp rostime tf std_msgs sensor_msgs nodelet cv_bridge compressed_depth_image_transport kinect2_registration bb_kinect2_msgs)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

find_package( PCL 1.8 REQUIRED )

################################################
## Declare ROS messages, services and actions ##
################################################

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
  CATKIN_DEPENDS kinect2_registration
)

###########
## Build ##
###########

if (UNIX)
	# Display additional files in qtcreator
	execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f OUTPUT_VARIABLE FILES_STRING)
	string(REPLACE "\n" ";" FILES_LIST ${FILES_STRING})
	add_custom_target(additional_files_${PROJECT_NAME}
	  SOURCES
	  ${FILES_LIST}
	  ${PROJECT_SOURCE_DIR}/../README.md
	)
endif()

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${freenect2_INCLUDE_DIRS}
  ${kinect2_registration_INCLUDE_DIRS}
)

if (WIN32)
    include_directories(${KinectSDK2_INCLUDE_DIRS}
                        ${Qt5Core_INCLUDE_DIRS}
                        ${Qt5WinExtras_INCLUDE_DIRS}
                        ${PCL_INCLUDE_DIRS})

    add_definitions(${PCL_DEFINITIONS})

    link_directories(${PCL_LIBRARY_DIRS})
endif()

if(DEPTH_REG_OPENCL)
  add_definitions(-DDEPTH_REG_OPENCL)
endif()
if(DEPTH_REG_CPU)
  add_definitions(-DDEPTH_REG_CPU)
endif()
add_definitions(-DK2_CALIB_PATH="${PROJECT_SOURCE_DIR}/data/")

set(kinect2_bridge_HEADERS include/kinect2_bridge/kinect2_bridge.h)
set(kinect2_bridge_SOURCES src/kinect2_bridge.cpp
                           src/kinect2_bridge_main.cpp)

if (WIN32)
    list(APPEND kinect2_bridge_SOURCES src/kinect2_bridge_windows.cpp)
else()
    list(APPEND kinect2_bridge_SOURCES src/kinect2_bridge_linux.cpp)
endif()

add_library(kinect2_bridge_nodelet SHARED ${kinect2_bridge_HEADERS} ${kinect2_bridge_SOURCES})
target_link_libraries(kinect2_bridge_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${freenect2_LIBRARY}
  ${kinect2_registration_LIBRARY}
  ${KinectSDK2_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5WinExtras_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(kinect2_bridge ${kinect2_bridge_HEADERS} ${kinect2_bridge_SOURCES})
target_link_libraries(kinect2_bridge
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${freenect2_LIBRARY}
  ${kinect2_registration_LIBRARY}
  ${KinectSDK2_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5WinExtras_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS kinect2_bridge kinect2_bridge_nodelet
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES
  nodelet_plugins.xml
  # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kinect2_bridge.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
