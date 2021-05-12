cmake_minimum_required(VERSION 3.5.2)
project(sim_gazebo_plugins)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  # cmake_modules
  roscpp 
  std_msgs 
  std_srvs 
  geometry_msgs 
  nav_msgs tf 
  dynamic_reconfigure 
  message_generation
  octomap_msgs
  octomap_ros
)
include_directories(include ${catkin_INCLUDE_DIRS})

## Find gazebo
find_package(gazebo REQUIRED)
# if(gazebo_VERSION_MAJOR GREATER 8)
#   ign_import_target(UUID)
#   message(STATUS "Applying patch to ${PROJECT_NAME} to avoid error with GZ9: Target UUID::UUID not found.")
# endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# find_package(cmake_modules REQUIRED)
find_package(octomap REQUIRED)

find_package(Eigen3 REQUIRED)

# Declare the service files to be built
add_service_files(
  FILES
  Octomap.srv
)

generate_messages(DEPENDENCIES geometry_msgs octomap_msgs std_msgs)

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
    CATKIN_DEPENDS octomap_msgs octomap_ros roscpp std_msgs geometry_msgs nav_msgs tf message_runtime
    DEPENDS gazebo octomap EIGEN3
    INCLUDE_DIRS include
    LIBRARIES
)

###########
## Build ##
###########

add_library(BuildOctomapPlugin SHARED src/plugin_build_octomap.cpp)
target_link_libraries(BuildOctomapPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(BuildOctomapPlugin ${sim_gazebo_plugins_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

include_directories(SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  BuildOctomapPlugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
