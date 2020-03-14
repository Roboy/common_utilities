cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project( common_utilities )


SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g")

SET(CMAKE_BUILD_TYPE Debug)

#==============================================================================
# Dependencies
#==============================================================================
#find_package(catkin REQUIRED roscpp std_msgs tf interactive_markers tf tf_conversions visualization_msgs roboy_middleware_msgs)
#catkin_package(CATKIN_DEPENDS message_runtime INCLUDE_DIRS include
#        LIBRARIES rviz_visualization UDPSocket MotorConfig)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs  REQUIRED)
find_package(roboy_middleware_msgs REQUIRED)

find_package(eigen3_cmake_module)
find_package(Eigen3)
include_directories(${Eigen3_INCLUDE_DIRS})

# yaml-cpp
find_package(yaml-cpp REQUIRED yaml-cpp)
include_directories(${yaml-cpp_INCLUDE_DIRS})

include_directories( include )
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

add_library(motor_config SHARED src/MotorConfig.cpp) # include/common_utilities/MotorConfig.hpp include/common_utilities/CommonDefinitions.h)
#target_compile_definitions(motor_config)
ament_target_dependencies(motor_config rclcpp rclcpp_components std_msgs roboy_middleware_msgs})
#==============================================================================
# Application
#==============================================================================
#add_library(UDPSocket src/UDPSocket.cpp)
#target_link_libraries( UDPSocket ${catkin_LIBRARIES}  )
#
#add_executable( ROS_MASTER_URI_broadcaster src/ROS_MASTER_URI_broadcaster.cpp)
#target_link_libraries( ROS_MASTER_URI_broadcaster ${catkin_LIBRARIES} UDPSocket )
#
#add_executable( ROS_MASTER_URI_receiver src/ROS_MASTER_URI_receiver.cpp)
#target_link_libraries( ROS_MASTER_URI_receiver ${catkin_LIBRARIES} UDPSocket )

#add_library( rviz_visualization src/rviz_visualization.cpp )
#target_link_libraries( rviz_visualization ${catkin_LIBRARIES} )

#add_library( MotorConfig src/MotorConfig.cpp )
#add_dependencies( MotorConfig roboy_middleware_msgs_gencpp)
#target_link_libraries( MotorConfig ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES} )

#add_executable( rfid_unlocker src/rfid_unlocker.cpp )
#target_link_libraries( rfid_unlocker ${catkin_LIBRARIES} UDPSocket )

#==============================================================================
# Installation
#==============================================================================
#install(TARGETS
#        ROS_MASTER_URI_broadcaster
#        ROS_MASTER_URI_receiver
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )

install(TARGETS motor_config
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )


install(TARGETS
        motor_config
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        )



#install(DIRECTORY include/common_utilities/
#        DESTINATION include
#        FILES_MATCHING PATTERN "*.hpp"
#        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
        PATTERN ".git" EXCLUDE)

ament_export_dependencies(std_msgs rclcpp roboy_middleware_msgs roboy_control_msgs)
ament_export_include_directories(include)
ament_export_libraries(motor_config)

ament_package()