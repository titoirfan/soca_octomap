cmake_minimum_required(VERSION 3.0.2)
project(soca_octomap)

find_package(catkin REQUIRED COMPONENTS
  octomap_msgs
  octomap_ros
  roscpp
  visualization_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  dynamic_reconfigure
  pcl_ros
  pcl_conversions
)

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

generate_dynamic_reconfigure_options(cfg/OctomapDoseServer.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES soca_octomap
  CATKIN_DEPENDS octomap_msgs roscpp std_msgs std_srvs
  DEPENDS OCTOMAP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_library(octomap_dose_server src/OctomapDoseServer.cpp src/OcTreeDose.cpp)
target_link_libraries(octomap_dose_server ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})
add_dependencies(octomap_dose_server ${PROJECT_NAME}_gencfg)

add_executable(coverage_eval_node scripts/coverage_eval_node.cpp)
add_dependencies(coverage_eval_node ${coverage_eval_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(coverage_eval_node ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(heatmap_eval_node scripts/heatmap_eval_node.cpp src/OcTreeDose.cpp)
add_dependencies(heatmap_eval_node ${heatmap_eval_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(heatmap_eval_node ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(octomap_dose_server_node scripts/octomap_dose_server_node.cpp src/OctomapDoseServer.cpp src/OcTreeDose.cpp)
add_dependencies(octomap_dose_server_node ${octomap_dose_server_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(octomap_dose_server_node ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(octomap_dose_saver_node scripts/octomap_dose_saver_node.cpp src/OcTreeDose.cpp)
add_dependencies(octomap_dose_saver_node ${octomap_dose_saver_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(octomap_dose_saver_node ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})