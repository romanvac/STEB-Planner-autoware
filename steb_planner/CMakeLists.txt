cmake_minimum_required(VERSION 3.14)
project(steb_planner)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

message(STATUS "System: ${CMAKE_SYSTEM}")
## System dependencies are found with CMake's conventions
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${CMAKE_MODULE_PATH}")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)
find_package(glog REQUIRED)
find_package(Boost REQUIRED)

# Eigen3 FindScript Backward compatibility (ubuntu saucy)
# Since FindEigen.cmake is deprecated starting from jade.
if (EXISTS "FindEigen3.cmake")
    find_package(Eigen3 REQUIRED)
    set(Eigen_INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS})
elseif (EXISTS "FindEigen.cmake")
    find_package(Eigen REQUIRED)
elseif (EXISTS "FindEigen.cmake")
    message(WARNING "No findEigen cmake script found. You must provde one of them,
  e.g. by adding it to ${PROJECT_SOURCE_DIR}/cmake_modules.")
endif (EXISTS "FindEigen3.cmake")

set(EXTERNAL_INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${Boost_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES} ${OpenCV_LIBRARIES} glog::glog)

###########
## Build ##
###########
include_directories(
        include
        ${Boost_INCLUDE_DIRS}
        ${EXTERNAL_INCLUDE_DIRS}
)

ament_auto_add_library(steb_planner SHARED
  # src/steb_planner_node.cpp
  src/steb_planner/steb_optimal/steb_config.cpp
  src/steb_planner/steb_optimal/saptio_temporal_elastic_band.cpp
  src/steb_planner/steb_optimal/steb_planner.cpp
)

target_include_directories(steb_planner
  SYSTEM PUBLIC
#    ${OpenCV_INCLUDE_DIRS}
#    ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${EXTERNAL_INCLUDE_DIRS}
)

target_link_libraries(steb_planner
#  ${OpenCV_LIBRARIES}
        ${EXTERNAL_LIBS}
)

# rclcpp_components_register_node(steb_planner
#   PLUGIN "STEBPlanner"
#   EXECUTABLE steb_planner_node
# )

ament_auto_add_executable(steb_planner_node src/steb_planner_node.cpp)
add_dependencies(steb_planner_node steb_planner)
target_link_libraries(steb_planner_node steb_planner)


ament_auto_add_executable(steb_planner_test src/steb_planner_node.cpp)
add_dependencies(steb_planner_test steb_planner)
target_link_libraries(steb_planner_test  steb_planner)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
    rviz
)
