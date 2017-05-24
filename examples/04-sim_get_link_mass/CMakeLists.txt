# add the include directory to the include paths
INCLUDE_DIRECTORIES (
	${SAI2-COMMON_INCLUDE_DIRS})

# create an executable
ADD_EXECUTABLE (04-sim_get_link_mass 04-sim_get_link_mass.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (04-sim_get_link_mass
	${SAI2-COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY world.urdf pbot.urdf DESTINATION ${RESOURCE_DIR})
