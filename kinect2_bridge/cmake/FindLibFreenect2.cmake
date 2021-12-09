# - Find libfreenect
# 
# If the LibFreenect2_ROOT environment variable
# is defined, it will be used as base path.
# The following standard variables get defined:
#  LibFreenect2_FOUND:    true if LibUSB was found
#  LibFreenect2_INCLUDE_DIR: the directory that contains the include file
#  LibFreenect2_LIBRARIES:  the libraries

FIND_PATH(LibFreenect2_INCLUDE_DIRS
  NAMES libfreenect2.hpp
  PATHS
	"$ENV{ROS_ROOT}/../../include/libfreenect2"
	"$ENV{ROS_ROOT}/include/libfreenect2"    
    ENV LibFreenect2_ROOT
  PATH_SUFFIXES
    libfreenect2
)

SET(LibFreenect2_NAME freenect2 freenect2d)

FIND_LIBRARY(LibFreenect2_LIBRARIES
  NAMES ${LibFreenect2_NAME}
  PATHS
	"$ENV{ROS_ROOT}/../../lib"
	"$ENV{ROS_ROOT}/lib"
    ENV LibFreenect2_ROOT
)

IF(WIN32)
FIND_FILE(LibFreenect2_DLL
  ${LibFreenect2_NAME}.dll
  PATHS
	"$ENV{ROS_ROOT}/../../bin"
	"$ENV{ROS_ROOT}/bin"
    ENV LibFreenect2_ROOT
)
ENDIF()

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LibFreenect2 FOUND_VAR LibFreenect2_FOUND
  REQUIRED_VARS LibFreenect2_LIBRARIES LibFreenect2_INCLUDE_DIRS)
