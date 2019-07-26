cmake_minimum_required(VERSION 2.8.3)
project(lslidar_c32_driver)

add_definitions(-std=c++0x)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  diagnostic_updater
  nodelet

  lslidar_c32_msgs
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lslidar_c32_driver
  CATKIN_DEPENDS
    roscpp diagnostic_updater nodelet
    lslidar_c32_msgs
  DEPENDS
    boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

# Leishen c32 lidar driver
add_library(lslidar_c32_driver
  src/lslidar_c32_driver.cc
)
target_link_libraries(lslidar_c32_driver
  ${catkin_LIBRARIES}
)
add_dependencies(lslidar_c32_driver
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Leishen c32 lidar nodelet
add_executable(lslidar_c32_driver_node
  src/lslidar_c32_driver_node.cc
)
target_link_libraries(lslidar_c32_driver_node
  lslidar_c32_driver
  ${catkin_LIBRARIES}
)
add_dependencies(lslidar_c32_driver_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# install options
install(TARGETS lslidar_c32_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

