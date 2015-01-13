cmake_minimum_required(VERSION 2.8.3)
project(fast_simulator2)

find_package(catkin REQUIRED COMPONENTS
    ed
    ed_object_models
    urdf
    kdl_parser
    rgbd
    class_loader
)

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
# add_message_files(
#    FILES
#    message1.msg
#    ...
# )

# Generate services
# add_service_files(
#    FILES
#    service1.srv
#    ...
# )

# Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#    ...
# )

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fast_simulator2
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

set(HEADER_FILES
    include/fast_simulator2/simulator.h
    include/fast_simulator2/types.h
    include/fast_simulator2/plugin.h
)

add_library(fast_simulator2
    src/simulator.cpp
    src/plugin_container.cpp
    ${HEADER_FILES}
)
target_link_libraries(fast_simulator2 ${catkin_LIBRARIES})

add_executable(sim2
    src/main.cpp
)
target_link_libraries(sim2 fast_simulator2)

# ------------------------------------------------------------------------------------------------
#                                              PLUGINS
# ------------------------------------------------------------------------------------------------

add_library(sim_base_controller plugins/base_controller.cpp)
target_link_libraries(sim_base_controller fast_simulator2)

add_library(sim_depth_sensor plugins/depth_sensor_plugin.cpp)
target_link_libraries(sim_depth_sensor fast_simulator2)

add_library(sim_ros_robot plugins/ros_robot_plugin.cpp)
target_link_libraries(sim_ros_robot fast_simulator2)

