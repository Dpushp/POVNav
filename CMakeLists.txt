cmake_minimum_required(VERSION 2.8.3)
project(pov_nav)

set (MALLOC_CHECK_ 2)
set (OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
	OpenCV REQUIRED
    roscpp
    rospy
    sensor_msgs
    geometry_msgs
    std_msgs
    cv_bridge
    image_transport
    dynamic_reconfigure
    )


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/pov_nav_dyparam.cfg
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dynamic_tutorials
  CATKIN_DEPENDS dynamic_reconfigure roscpp rospy
#  DEPENDS system_lib
)


## Build ball_detection
include_directories( include ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )

file(GLOB UTILITY_FILE
    src/pov_nav.cpp
    src/pov_plan.cpp
    src/pid_IBVS.cpp
)

add_executable(sim_pov_nav  
    ${UTILITY_FILE}
    src/pov_nav_node.cpp)
# make sure configure headers are built before any node using them
add_dependencies(sim_pov_nav ${PROJECT_NAME}_gencfg)
target_link_libraries(sim_pov_nav 
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES})
