CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12)

PROJECT (SAI2-COMMON)
SET (PROJECT_VERSION 0.1.0) #major version is zero for now since api will certainly change in future

SET(CMAKE_CXX_FLAGS "-std=c++11")

# include Chai3d (required by Dynamics3D)
# -- this is required by Chai3d to pick the correct path for OpenGL headers
FIND_PACKAGE(CHAI3D REQUIRED)
ADD_DEFINITIONS(${CHAI3D_DEFINITIONS})
if (CMAKE_SYSTEM_NAME MATCHES Darwin) 
  ADD_DEFINITIONS(-D_MACOSX -DMACOSX)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
  ADD_DEFINITIONS(-D_LINUX -DLINUX)
endif () #NOTE: Windows is currently unsupported
LINK_DIRECTORIES(${CHAI3D_LIBRARY_DIRS})

# include Sai2-Simulation
FIND_PACKAGE(SAI2-SIMULATION REQUIRED)

# include Parser
SET(PARSER_SOURCE ${PROJECT_SOURCE_DIR}/src/parser/URDFToDynamics3d.cpp
	${PROJECT_SOURCE_DIR}/src/parser/URDFToChaiGraphics.cpp)

SET (SAI2-COMMON_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src)

# include urdf, compiled with application
SET(URDF_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src/parser/)
SET(URDF_SOURCE ${URDF_INCLUDE_DIR}/urdf/urdfdom/urdf_parser/src/pose.cpp
		${URDF_INCLUDE_DIR}/urdf/urdfdom/urdf_parser/src/link.cpp
		${URDF_INCLUDE_DIR}/urdf/urdfdom/urdf_parser/src/graphics.cpp
		${URDF_INCLUDE_DIR}/urdf/urdfdom/urdf_parser/src/world.cpp
		${URDF_INCLUDE_DIR}/urdf/urdfdom/urdf_parser/src/model.cpp
		${URDF_INCLUDE_DIR}/urdf/urdfdom/urdf_parser/src/joint.cpp
		${URDF_INCLUDE_DIR}/urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h
		${URDF_INCLUDE_DIR}/urdf/urdfdom_headers/urdf_exception/include/urdf_exception/exception.h
		${URDF_INCLUDE_DIR}/urdf/urdfdom_headers/urdf_model/include/urdf_model/pose.h
		${URDF_INCLUDE_DIR}/urdf/urdfdom_headers/urdf_world/include/urdf_world/world.h
		${URDF_INCLUDE_DIR}/urdf/urdfdom_headers/urdf_model/include/urdf_model/model.h
		${URDF_INCLUDE_DIR}/urdf/urdfdom_headers/urdf_model/include/urdf_model/link.h
		${URDF_INCLUDE_DIR}/urdf/urdfdom_headers/urdf_model/include/urdf_model/joint.h
		${URDF_INCLUDE_DIR}/urdf/boost_replacement/lexical_cast.h
		${URDF_INCLUDE_DIR}/urdf/boost_replacement/shared_ptr.h
		${URDF_INCLUDE_DIR}/urdf/boost_replacement/printf_console.cpp
		${URDF_INCLUDE_DIR}/urdf/boost_replacement/printf_console.h
		${URDF_INCLUDE_DIR}/urdf/boost_replacement/string_split.cpp
		${URDF_INCLUDE_DIR}/urdf/boost_replacement/string_split.h)

# include Simulation
SET(SIMULATION_SOURCE ${PROJECT_SOURCE_DIR}/src/simulation/SimulationInterface.cpp
	${PROJECT_SOURCE_DIR}/src/simulation/Sai2Simulation.cpp)

# include Graphics
SET(GRAPHICS_SOURCE ${PROJECT_SOURCE_DIR}/src/graphics/GraphicsInterface.cpp
	${PROJECT_SOURCE_DIR}/src/graphics/ChaiGraphics.cpp)

# include Model
SET(MODEL_SOURCE ${PROJECT_SOURCE_DIR}/src/model/ModelInterface.cpp
	${PROJECT_SOURCE_DIR}/src/model/RBDLModel.cpp)

# Add the include directory to the include paths
INCLUDE_DIRECTORIES (
	${PROJECT_SOURCE_DIR}/src
	${URDF_INCLUDE_DIR}
	${CHAI3D_INCLUDE_DIRS}
	${SAI2-SIMULATION_INCLUDE_DIRS}
	)

# Create the library
ADD_LIBRARY (sai2-common STATIC
	${URDF_SOURCE}
	${PARSER_SOURCE}
	${SIMULATION_SOURCE}
	${GRAPHICS_SOURCE}
	${MODEL_SOURCE}
	)

FIND_LIBRARY(TINYXML_LIB tinyxml2)
FIND_LIBRARY(RBDL_LIB rbdl )
FIND_LIBRARY(RBDL_URDF_LIB rbdl_urdfreader)
SET (SAI2-COMMON_LIBRARIES sai2-common 
	${TINYXML_LIB}
	${CHAI3D_LIBRARIES}
	${SAI2-SIMULATION_LIBRARIES}
	${RBDL_LIB} ${RBDL_URDF_LIB}
	)

# 
# export package
# 

# export package for use from the build tree
EXPORT (TARGETS sai2-common FILE ${PROJECT_BINARY_DIR}/SAI2-COMMONTargets.cmake)
EXPORT (PACKAGE SAI2-COMMON)

# configure package
SET (CONF_INCLUDE_DIRS ${SAI2-COMMON_INCLUDE_DIRS})
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
