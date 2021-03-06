CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12)

PROJECT (SAI2-COMMON)
SET (PROJECT_VERSION 0.1.0) #major version is zero for now since api will certainly change in future

SET(CMAKE_CXX_FLAGS "-std=c++11 -I/usr/include -I/usr/local/include")

# set default build to release with debug info. mimic'ed from RBDL project.
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	MESSAGE(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")
	# Set the default build, and also the possible values of build type for cmake-gui
	SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
	SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()

# include Chai3d 
# -- this is required by Chai3d to pick the correct path for OpenGL headers
FIND_PACKAGE(CHAI3D REQUIRED)
ADD_DEFINITIONS(${CHAI3D_DEFINITIONS})
if (CMAKE_SYSTEM_NAME MATCHES Darwin) 
  ADD_DEFINITIONS(-D_MACOSX -DMACOSX)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
  ADD_DEFINITIONS(-D_LINUX -DLINUX)
endif () #NOTE: Windows is currently unsupported
LINK_DIRECTORIES(${CHAI3D_LIBRARY_DIRS})

#include Sai2-Urdf
FIND_PACKAGE(SAI2-URDF REQUIRED)

# include Sai2-Simulation
FIND_PACKAGE(SAI2-SIMULATION REQUIRED)

# include Sai2-Model
FIND_PACKAGE(SAI2-MODEL REQUIRED)

# include Sai2-Graphics
FIND_PACKAGE(SAI2-GRAPHICS REQUIRED)
ADD_DEFINITIONS(${SAI2-GRAPHICS_DEFINITIONS})

# include Eigen
FIND_PACKAGE(Eigen3 REQUIRED)

# include tinyxml2
FIND_LIBRARY(TINYXML_LIBRARIES tinyxml2)

# jsoncpp
if (CMAKE_SYSTEM_NAME MATCHES Darwin) 
	find_library(JSONCPP_LIBRARY jsoncpp)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
	find_package(PkgConfig REQUIRED)
	pkg_check_modules(JSONCPP jsoncpp)
	link_directories(${JSONCPP_LIBRARY_DIRS})
	set(JSONCPP_LIBRARY ${JSONCPP_LIBRARIES})
endif ()


SET (SAI2-COMMON_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src)

# include Force Sensor
SET(FORCE_SENSOR_SOURCE ${PROJECT_SOURCE_DIR}/src/force_sensor/ForceSensorSim.cpp
	${PROJECT_SOURCE_DIR}/src/force_sensor/ForceSensorDisplay.cpp)

# include Redis
SET(REDIS_SOURCE ${PROJECT_SOURCE_DIR}/src/redis/RedisClient.cpp)

# include Timer
SET(TIMER_SOURCE ${PROJECT_SOURCE_DIR}/src/timer/LoopTimer.cpp)

# include filter
SET(FILTER_SOURCE ${PROJECT_SOURCE_DIR}/src/filters/ButterworthFilter.cpp)

# include UI force Widget
SET(UIFORCE_SOURCE ${PROJECT_SOURCE_DIR}/src/uiforce/UIForceWidget.cpp)

# Add the include directory to the include paths
INCLUDE_DIRECTORIES (
	${PROJECT_SOURCE_DIR}/src
	${CHAI3D_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${SAI2-URDF_INCLUDE_DIRS}
	${SAI2-SIMULATION_INCLUDE_DIRS}
	${SAI2-MODEL_INCLUDE_DIRS}
	${SAI2-GRAPHICS_INCLUDE_DIRS}
	${JSONCPP_INCLUDE_DIRS}
	)

# Create the library. 
ADD_LIBRARY (sai2-common STATIC
	${FORCE_SENSOR_SOURCE}
	${REDIS_SOURCE}
	${TIMER_SOURCE}
	${FILTER_SOURCE}
	${UIFORCE_SOURCE}
	)

SET (SAI2-COMMON_LIBRARIES sai2-common 
	${CHAI3D_LIBRARIES}
	${SAI2-URDF_LIBRARIES}
	${SAI2-SIMULATION_LIBRARIES}
	${SAI2-MODEL_LIBRARIES}
	${SAI2-GRAPHICS_LIBRARIES}
	${JSONCPP_LIBRARY}
	)

# 
# export package
# 

# export package for use from the build tree
EXPORT (TARGETS sai2-common FILE ${PROJECT_BINARY_DIR}/SAI2-COMMONTargets.cmake)
EXPORT (PACKAGE SAI2-COMMON)

# configure package
SET (CONF_INCLUDE_DIRS ${SAI2-COMMON_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS})
SET (CONF_LIBRARIES ${SAI2-COMMON_LIBRARIES})

# create package configuration and version files
CONFIGURE_FILE(SAI2-COMMONConfig.cmake.in "${PROJECT_BINARY_DIR}/SAI2-COMMONConfig.cmake" @ONLY)
INCLUDE(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SAI2-COMMONConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

# add examples
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/examples)
