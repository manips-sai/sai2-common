
# include GLFW3
FIND_PACKAGE(glfw3 QUIET)

# add the include directory to the include paths
INCLUDE_DIRECTORIES (
	${SAI2-COMMON_INCLUDE_DIRS})

# create an executable
ADD_EXECUTABLE (00-test_allegro 00-test_allegro.cpp)

# and link the library against the executable
FIND_LIBRARY(GLFW_LIB glfw)
TARGET_LINK_LIBRARIES (00-test_allegro
	${SAI2-COMMON_LIBRARIES}
	${GLFW_LIB})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY world.urdf allegro_hand_description_right.urdf DESTINATION ${RESOURCE_DIR})
