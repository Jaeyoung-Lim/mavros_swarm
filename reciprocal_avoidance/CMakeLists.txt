cmake_minimum_required(VERSION 2.8.3)
project(reciprocal_avoidance)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/single_vehicle.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# BINARIES #
############
cs_add_executable(single_vehicle_node
  src/single_vehicle_node.cpp
)
target_link_libraries(single_vehicle_node ${PROJECT_NAME} ${catkin_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()