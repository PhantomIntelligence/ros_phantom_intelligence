cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(phantom_intelligence)

#### Names #####################################
set(EXTERNAL_GATEWAY_LIB_NAME SpiritSensorGateway)
set(GATEWAY_DRIVER_LIB gateway_driver)
set(AWL16_NODE phantom_intelligence_awl16_node)
set(TEST_EXECUTABLE ${PROJECT_NAME}_test)
################################################

include(cmake_module/setupCMakeMacros.cmake)
setup_cmake_macros(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

######################################################################
# Complete instructions on how to build this file are present at:    #
#       http://wiki.ros.org/catkin/CMakeLists.txt                    #
######################################################################

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

######### catkin macro and libraries ##################################################
set(${PROJECT_NAME}_CATKIN_DEPS
        roscpp
        message_generation
        std_msgs
        )

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
#######################################################################################

######### ROS messages and services ###################################################
add_message_files(
        DIRECTORY msg
        FILES
        PhantomAWL.msg
)

add_service_files(
        DIRECTORY srv
        FILES SendCommandToSensor.srv
)

generate_messages(DEPENDENCIES std_msgs)
#######################################################################################

######### catkin specific configuration ###############################################
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${GATEWAY_DRIVER_LIB}
        CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
        DEPENDS ${EXTERNAL_GATEWAY_LIB_NAME}
)
#######################################################################################

include_directories(include ${catkin_INCLUDE_DIRS})

##### Driver Gateway Lib ##############################################################
include(cmake_module/SpiritSensorGateway.cmake)
fetchSpiritSensorGateway(${PROJECT_SOURCE_DIR}/cmake_module
        ${PROJECT_BINARY_DIR}/lib/${EXTERNAL_GATEWAY_LIB_NAME})

file(GLOB GATEWAY_DRIVER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/driver/gateway/*")

add_library(${GATEWAY_DRIVER_LIB} ${GATEWAY_DRIVER_FILES})
add_dependencies(${GATEWAY_DRIVER_LIB} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${GATEWAY_DRIVER_LIB}
        ${EXTERNAL_GATEWAY_LIB_NAME}
        ${catkin_LIBRARIES}
        )

#######################################################################################

add_executable(${AWL16_NODE}
        ${GATEWAY_DRIVER_LIB}
        ${catkin_LIBRARIES}
        )

## Mark executables and libraries for installation
install(TARGETS
        ${AWL16_NODE}
        ${GATEWAY_DRIVER_LIB}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)
install(
        DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


if (CATKIN_ENABLE_TESTING)
    find_package(roslaunch REQUIRED)
    file(GLOB TEST_FILES ${CMAKE_CURRENT_SOURCE_DIR}/test/*)

    # For the test file formatting, see:
    # https://github.com/ros-drivers/omron/tree/indigo-devel/omron_os32c_driver/test
    # TODO: remove this comment once a test is done
    catkin_add_gtest(${TEST_EXECUTABLE} ${TEST_FILES})

    # parse check all the launch/*.launch files
    roslaunch_add_file_check(launch)
    target_link_libraries(${TEST_EXECUTABLE} ${catkin_LIBRARIES})
endif ()

