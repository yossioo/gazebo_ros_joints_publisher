cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_joints_publisher)

add_compile_options(-std=c++11)
cmake_policy(SET CMP0054 NEW)

find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

#[[
message("------------ BOOST_VERSION: ${Boost_LIB_VERSION}")
message("------------ Boost_FOUND: ${Boost_FOUND}")
message("------------ CMAKE_VERSION: ${CMAKE_VERSION}")
]]

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS}
        # ${Qt5Core_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR})
link_directories(${GAZEBO_LIBRARY_DIRS})

#include_directories( ${catkin_INCLUDE_DIRS}  ${ADDITIONAL_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS})
if (${gazebo_VERSION_MAJOR} GREATER 8)
    add_definitions("-DBOOST_VARIANT_USE_RELAXED_GET_BY_DEFAULT")
    link_directories(${Boost_LIBRARY_DIRS})
    cmake_minimum_required(VERSION 3.5.1)
endif ()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
#########

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        gazebo_ros
        geometry_msgs
        gazebo_plugins
        dynamic_reconfigure
        control_toolbox
        )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS
        system
        )


catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES angry_birds_simulator
        #  CATKIN_DEPENDS roscpp std_msgs
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
