cmake_minimum_required(VERSION 2.8.7)
project(semantic_mapping)

IF(NOT CMAKE_BUILD_TYPE)
 SET(CMAKE_BUILD_TYPE Release)
ENDIF()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3  -march=native -Wno-reorder")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native -Wno-reorder")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

IF(DEFINED ENV{ARM_ARCHITECTURE})
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
ELSE()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3 -msse4")
ENDIF()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

#set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1)

find_package(OpenCV 3.1 QUIET)

if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.13 QUIET)
    message(STATUS "OpenCV 2.4.13 .")
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()
find_package(Eigen3 3.1.0 REQUIRED)
find_package( Ceres REQUIRED )

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    eigen_conversions
    geometry_msgs
    message_generation
    std_msgs
    message_filters
    image_transport
    obstacle_msgs
    cv_bridge
    tf
    rosbag_storage
    visualization_msgs
)

include_directories(
${catkin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${CERES_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME} obstacle_msgs
        CATKIN_DEPENDS roscpp std_msgs message_runtime tf visualization_msgs
        # DEPENDS system_lib
)


set(LIBS
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${CERES_LIBRARIES}
)

#add_subdirectory(src)
# core

add_executable( semantic_mapping_dual_node
        src/semantic_mapping_dual_node.cpp
        src/map_add.cpp
        src/obstacle_ind_merge.cpp
        src/map_publisher.cpp
        src/map_element.cpp
        src/obstacle_merge.cpp)

target_link_libraries( semantic_mapping_dual_node
        ${LIBS}
        )

add_dependencies(semantic_mapping_dual_node
        obstacle_msgs_generate_messages_cpp)

add_executable( semantic_mapping_node
        src/semantic_mapping_node.cpp
        src/map_add.cpp
        src/obstacle_ind_merge.cpp
        src/map_publisher.cpp
        src/map_element.cpp
        src/obstacle_merge.cpp)

target_link_libraries( semantic_mapping_node
        ${LIBS}
        )

add_dependencies(semantic_mapping_node
        obstacle_msgs_generate_messages_cpp)
