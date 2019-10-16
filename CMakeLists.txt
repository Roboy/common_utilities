cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project( common_utilities )


SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

SET(CMAKE_BUILD_TYPE Debug)

#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED roscpp roslib std_msgs tf interactive_markers tf tf_conversions visualization_msgs roboy_middleware_msgs)
catkin_package(CATKIN_DEPENDS message_runtime roslib INCLUDE_DIRS include
        LIBRARIES rviz_visualization UDPSocket MotorConfig)

# Eigen
find_package(PkgConfig REQUIRED )
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

# yaml-cpp
pkg_check_modules( yaml-cpp REQUIRED yaml-cpp)
include_directories(${yaml-cpp_INCLUDE_DIRS})

include_directories( include ${catkin_INCLUDE_DIRS} )

#==============================================================================
# Application
#==============================================================================
add_library(UDPSocket src/UDPSocket.cpp)
target_link_libraries( UDPSocket ${catkin_LIBRARIES}  )

add_executable( ROS_MASTER_URI_broadcaster src/ROS_MASTER_URI_broadcaster.cpp)
target_link_libraries( ROS_MASTER_URI_broadcaster ${catkin_LIBRARIES} UDPSocket )

add_executable( ROS_MASTER_URI_receiver src/ROS_MASTER_URI_receiver.cpp)
target_link_libraries( ROS_MASTER_URI_receiver ${catkin_LIBRARIES} UDPSocket )

add_library( rviz_visualization src/rviz_visualization.cpp )
target_link_libraries( rviz_visualization ${catkin_LIBRARIES} )

add_library( MotorConfig src/MotorConfig.cpp )
add_dependencies( MotorConfig roboy_middleware_msgs_gencpp)
target_link_libraries( MotorConfig ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES} )

add_executable( rfid_unlocker src/rfid_unlocker.cpp )
target_link_libraries( rfid_unlocker ${catkin_LIBRARIES} UDPSocket )

#==============================================================================
# Installation
#==============================================================================
install(TARGETS
        ROS_MASTER_URI_broadcaster
        ROS_MASTER_URI_receiver
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(TARGETS
        UDPSocket
        rviz_visualization
        MotorConfig
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY include/common_utilities/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        )