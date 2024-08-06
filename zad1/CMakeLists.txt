cmake_minimum_required(VERSION 3.0.2)
project(zad1)

################################################################
## Find catkin macros and libraries and other catkin packages ##
################################################################
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    rrm_msgs
    urdf
    eigen_conversions
    tf
    tf_conversions
)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
include_directories(
    include
    include/zad1/client_gui
    include/zad1/geometry
    include/zad1/robot
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

###############################
## Declare a C++ executables ##
###############################
add_executable(${PROJECT_NAME}_robot
    src/robot/robot_node.cpp
    src/robot/robot.cpp
    src/robot/topic_publisher.cpp
    src/robot/service_servers.cpp
)
add_executable(${PROJECT_NAME}_client_gui
    src/client_gui/client_gui_node.cpp
    src/client_gui/graphic_client.cpp
)
add_executable(${PROJECT_NAME}_forward_kinematics
    src/geometry/geometry_node.cpp
    src/geometry/forward_kinematics.cpp
)

######################################################################
## Specify libraries to link a library or executable target against ##
######################################################################
target_link_libraries(${PROJECT_NAME}_robot ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_client_gui ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_forward_kinematics ${catkin_LIBRARIES})

#####################################################
## Add cmake target dependencies of the executable ##
#####################################################
add_dependencies(${PROJECT_NAME}_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_client_gui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_forward_kinematics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})