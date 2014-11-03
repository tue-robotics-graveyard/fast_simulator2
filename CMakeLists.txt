cmake_minimum_required(VERSION 2.8.3)
project(ed_simulator)

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
#  INCLUDE_DIRS include
#  LIBRARIES bla
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
    include/tue/simulator/types.h
    include/tue/simulator/world.h
    include/tue/simulator/plugin.h
    include/tue/simulator/object.h
    include/tue/simulator/update_request.h
    include/tue/simulator/id_map.h
)


add_library(ed_simulator_plugin
    src/simulator_plugin.cpp
    src/simulator.cpp
    src/object.cpp
    src/robot.cpp
    src/depth_sensor.cpp
    src/laser_range_finder.cpp
    src/world.cpp
    src/plugin_container.cpp
	src/update_request.cpp
    ${HEADER_FILES}
)
target_link_libraries(ed_simulator_plugin ${catkin_LIBRARIES})

add_executable(ed_simulator
    src/main.cpp
)
target_link_libraries(ed_simulator ed_simulator_plugin)

# ------------------------------------------------------------------------------------------------
#                                              PLUGINS
# ------------------------------------------------------------------------------------------------

add_library(sim_base_controller plugins/base_controller.cpp)
target_link_libraries(sim_base_controller ed_simulator_plugin)


