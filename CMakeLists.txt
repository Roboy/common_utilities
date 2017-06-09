cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project( common_utilities )


SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

SET(CMAKE_BUILD_TYPE Debug)

#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED roscpp std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories( include ${catkin_INCLUDE_DIRS} )

#==============================================================================
# Application
#==============================================================================
add_executable( IPbroadcaster src/UDPSocket.cpp src/IPbroadcaster.cpp )
target_link_libraries( IPbroadcaster ${catkin_LIBRARIES} )

add_executable( IPreceiver src/UDPSocket.cpp src/IPreceiver.cpp )
target_link_libraries( IPreceiver ${catkin_LIBRARIES} )

#==============================================================================
# Installation
#==============================================================================
install(TARGETS
        IPbroadcaster
        IPreceiver
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )