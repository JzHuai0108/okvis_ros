cmake_minimum_required(VERSION 2.8.3)
project(okvis_ros)

SET(CATKIN_DEVEL_PREFIX $ENV{HOME}/catkin_ws/devel) # this ensures binaries are put into correct location of catkin_ws

find_package(catkin REQUIRED  
#  pcl_conversions
#  pcl_ros
  roscpp
  rosbag
  tf
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_geometry
)

# generate dynamic reconfigure stuff
generate_dynamic_reconfigure_options(cfg/Camera.cfg)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS rosbag tf roscpp cv_bridge message_runtime std_msgs sensor_msgs
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)


set(CMAKE_BUILD_TYPE_SAVED "${CMAKE_BUILD_TYPE}")
# we really want to use Release here
set(CMAKE_BUILD_TYPE Release)

message(STATUS "Building okvis with ${CMAKE_BUILD_TYPE}")

# The configure step fails at catkin_package() if this directory
# doesn't exist yet. Create it at configure time!
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

# let's pull in the okvis plain cmake project
set(BUILD_SHARED_LIBS OFF)
add_subdirectory(okvis)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
find_package( Eigen REQUIRED )
include_directories(${EIGEN_INCLUDE_DIR})
# libvisensor
find_package(VISensor)

if(VISENSORDRIVER_FOUND)
  add_definitions(-DHAVE_LIBVISENSOR)
  message(STATUS "Found libvisensor in ROS interface.")
  include_directories(${VISensorDriver_INCLUDE_DIR})
else()
  message(WARNING "libvisensor not found")
endif()

find_package(vio_common REQUIRED)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}/include
  ${vio_common_INCLUDE_DIR}
)
message("okvis_ros cmake install prefix " ${CMAKE_INSTALL_PREFIX})
set(CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE_SAVED}")
message("Building okvis_ros " ${CMAKE_BUILD_TYPE})
# compile the ROS dependent stuff into a library
add_definitions(-fPIC -march=native -Wextra -Winit-self -Woverloaded-virtual
                -Wnon-virtual-dtor -Wsign-promo -Wno-long-long -std=c++0x)
add_library(${PROJECT_NAME} STATIC src/Subscriber.cpp
                               src/Player.cpp                            
                               src/Publisher.cpp
                               src/RosParametersReader.cpp
                               include/okvis/Subscriber.hpp
                               include/okvis/Publisher.hpp
                               include/okvis/RosParametersReader.hpp)

add_dependencies(${PROJECT_NAME} okvis_multisensor_processing)

# make sure configure headers are built before any node using them
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}  
  ${catkin_LIBRARIES}
  okvis_util
  okvis_kinematics
  okvis_time
  okvis_cv 
  okvis_common
  okvis_ceres
  okvis_timing
  okvis_matcher
  okvis_frontend 
  okvis_multisensor_processing
  pthread
  ${vio_common_LIBRARY}
)

if(VISENSORDRIVER_FOUND)
  target_link_libraries(${PROJECT_NAME} ${VISensorDriver_LIBRARY})
endif()

include_directories(
  okvis/okvis_util/include
  okvis/okvis_kinematics/include
  okvis/okvis_time/include
  okvis/okvis_cv/include
  okvis/okvis_common/include
  okvis/okvis_ceres/include
  okvis/okvis_timing/include
  okvis/okvis_matcher/include
  okvis/okvis_frontend/include 
  okvis/okvis_multisensor_processing/include
)

# now the actual applications
add_executable(okvis_node src/okvis_node.cpp )
target_link_libraries(okvis_node ${PROJECT_NAME} )
add_executable(okvis_node_synchronous src/okvis_node_synchronous.cpp)
target_link_libraries(okvis_node_synchronous ${PROJECT_NAME} )
add_executable(dataset_convertor src/dataset_convertor.cpp)
target_link_libraries(dataset_convertor ${PROJECT_NAME} )

