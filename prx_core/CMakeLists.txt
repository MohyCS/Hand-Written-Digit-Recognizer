#############################
## PRELIMINARIES
#############################

# Set the required version of CMake
cmake_minimum_required(VERSION 2.8.3)

# Define the project name
project(prx_core)

#############################
## SET ENVIRONMENT VARIABLES
#############################
option(PRX_BUILD "Debug, or Release compilation mode." Release)
# Set the build type.  Options are Coverage, Debug, Release, RelWitheDebInfo, MinSizeRel
set(CMAKE_BUILD_TYPE ${PRX_BUILD})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")

find_package( catkin REQUIRED 
              COMPONENTS roscpp rospy roslib message_generation geometry_msgs std_msgs class_loader tf actionlib_msgs actionlib)
find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED)
find_package( YamlCpp )
find_package( OpenSceneGraph REQUIRED
              COMPONENTS osgDB osgGA osgUtil osgViewer osgText)


# Assimp
find_path(ASSIMPINCLUDE assimp.h
    paths /usr/local/include/assimp /opt/local/include/assimp /usr/include/assimp
    doc "ASSIMP include directory")
find_library(ASSIMPLIB assimp
             paths /usr/local/lib /opt/local/lib /usr/lib
       doc "ASSIMP library")
if(ASSIMPINCLUDE AND ASSIMPLIB)
    message("-- Found ASSIMP header files")
else(ASSIMPINCLUDE AND ASSIMPLIB)
    message(WARNING "Could not find ASSIMP header files")
endif(ASSIMPINCLUDE AND ASSIMPLIB)


##
## MESSAGES, SERVICES AND ACTION TARGETS
##
file(GLOB_RECURSE MSG_UTIL RELATIVE ${PROJECT_SOURCE_DIR}/msg *.msg)
add_message_files(FILES ${MSG_UTIL})

file(GLOB_RECURSE SRV_UTIL RELATIVE ${PROJECT_SOURCE_DIR}/srv *.srv)
add_service_files(FILES ${SRV_UTIL})

# file(GLOB_RECURSE ACTION_MSGS RELATIVE ${PROJECT_SOURCE_DIR}/action *.action)
#
# add_action_files(
#  DIRECTORY action
#  FILES ${ACTION_MSGS}
#)

generate_messages(DEPENDENCIES geometry_msgs std_msgs actionlib_msgs)

catkin_package(
  INCLUDE_DIRS "prx"
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy roslib message_generation geometry_msgs std_msgs message_runtime class_loader tf actionlib
  DEPENDS Boost
  )

include_directories(${PROJECT_SOURCE_DIR})


# Catkin packages
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${YAMLCPP_INCLUDE_DIR})
include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})


# ASSIMP
if(ASSIMPINCLUDE)
    include_directories(${ASSIMPINCLUDE})
    add_definitions("-DOLD_ASSIMP")
endif(ASSIMPINCLUDE)

file(GLOB_RECURSE SRC_PRX ${PROJECT_SOURCE_DIR}/prx/*.cpp)

add_library(${PROJECT_NAME} ${SRC_PRX})

# target link libraries
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENSCENEGRAPH_LIBRARIES} ${YAMLCPP_LIBRARY} ${ASSIMPLIB} tinyxml2 )

# add dependency to the generation of messages

SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES ENABLE_EXPORTS TRUE)

add_dependencies( ${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp )

################
## Executable 
################
add_executable(util_test ${PROJECT_SOURCE_DIR}/nodes/util_main.cpp)
target_link_libraries(util_test ${PROJECT_NAME})
add_executable(vis_node ${PROJECT_SOURCE_DIR}/nodes/vis_main.cpp)
target_link_libraries(vis_node ${PROJECT_NAME})