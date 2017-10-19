cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project( common_utilities )


SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

SET(CMAKE_BUILD_TYPE Debug)

#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED roscpp std_msgs tf interactive_markers)
catkin_package(CATKIN_DEPENDS message_runtime INCLUDE_DIRS include)

# Eigen
find_package(PkgConfig REQUIRED )
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories( include ${catkin_INCLUDE_DIRS} )

#==============================================================================
# Application
#==============================================================================
add_library(UDPSocket src/UDPSocket.cpp)
target_link_libraries( UDPSocket ${catkin_LIBRARIES}  )

add_executable( IPbroadcaster src/IPbroadcaster.cpp )
target_link_libraries( IPbroadcaster ${catkin_LIBRARIES} UDPSocket )

add_executable( IPreceiver src/IPreceiver.cpp )
target_link_libraries( IPreceiver ${catkin_LIBRARIES} UDPSocket )

add_library( rviz_visualization src/rviz_visualization.cpp )
target_link_libraries( rviz_visualization ${catkin_LIBRARIES} )

#==============================================================================
# Installation
#==============================================================================
install(TARGETS
        IPbroadcaster
        IPreceiver
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(TARGETS
        UDPSocket
        rviz_visualization
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