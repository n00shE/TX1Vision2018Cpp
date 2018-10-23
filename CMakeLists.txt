SET(execName vision)
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
PROJECT(${execName})

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")

find_package(ZED 2 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)

IF(NOT WIN32)
     SET(SPECIAL_OS_LIBS "pthread" "X11")
    add_definitions(-Wno-write-strings)
ENDIF()

find_library(LibNT ntcore home/nvidia/Downloads/ntcore-3.1.7)

include_directories(${ZED_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_PATH})
include_directories(${GLEW_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${NTCORE_INCLUDE_DIR})
include_directories(/home/nvidia/Downloads/ntcore-3.1.7)
include_directories(/home/nvidia/Downloads/ntcore-3.1.7/include)
include_directories(/home/nvidia/Downloads/ntcore-3.1.7/wpiutil/include/)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
link_directories(${GLEW_LIBRARY_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(/home/nvidia/Downloads/ntcore-3.1.7)

FILE(GLOB_RECURSE SRC_FILES src/*)
FILE(GLOB_RECURSE HDR_FILES include/*.hpp)

ADD_EXECUTABLE(${execName} ${SRC_FILES}) #${HDR_FILES}
add_definitions(-std=c++11 -g -O3 -Intcore)

#configure_file (
#  "${PROJECT_SOURCE_DIR}/src/GripPipeline.cpp"
#  "${PROJECT_SOURCE_DIR}/src/GripPipeline.h"
#  "${PROJECT_BINARY_DIR}/TutorialConfig.h"
#  )
#include_directories("${PROJECT_BINARY_DIR}")
#add_executable(vision vision)

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${execName}
                        ${ZED_LIBRARIES}
                        ${SPECIAL_OS_LIBS}
                        ${GLEW_LIBRARIES}
                        ${OPENGL_LIBRARIES}
                        ${GLUT_LIBRARY}
                        ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED}
                        ${OpenCV_LIBS}
			"/home/nvidia/Downloads/ntcore-3.1.7")
