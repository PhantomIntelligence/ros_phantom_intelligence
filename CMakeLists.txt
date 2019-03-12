cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(phantom_intelligence)

#### Names #####################################
set(EXTERNAL_GATEWAY_LIB_NAME SensorGateway)
set(EXTERNAL_GATEWAY_DOWNLOAD_DIRECTORY "${PROJECT_BINARY_DIR}/lib/${EXTERNAL_GATEWAY_LIB_NAME}")
set(EXTERNAL_GATEWAY_INCLUDE_DIR "Defined in the fetchSensorGateway macro")
set(GATEWAY_DRIVER_LIB gateway_driver)
set(AWL16_NODE phantom_intelligence_awl16_node)
set(GUARDIAN_NODE phantom_intelligence_guardian_node)
set(TEST_EXECUTABLE ${PROJECT_NAME}_test)
################################################

include(cmake_module/setupCMakeMacros.cmake)
setup_ros_cmake_macros(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

fetchSensorGateway(${PROJECT_SOURCE_DIR}/cmake_module
        ${EXTERNAL_GATEWAY_DOWNLOAD_DIRECTORY})
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)

######################################################################
# Complete instructions on how to build this file are present at:    #
#       http://wiki.ros.org/catkin/CMakeLists.txt                    #
######################################################################

## Compile as C++14, 11 is supported in ROS Kinetic and newer, but 14 is required by SensorGateway
add_compile_options(-std=c++14)


######### catkin macro and libraries ##################################################
set(${PROJECT_NAME}_CATKIN_DEPS
        roscpp
        message_generation
        std_msgs
        gazebo_ros
        )

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
#######################################################################################
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(GAZEBO gazebo)
endif()
######### ROS messages and services ###################################################
add_message_files(
        DIRECTORY msg
        FILES
        PhantomIntelligenceHeader.msg
        PhantomIntelligenceRawData.msg
        PhantomIntelligenceFrame.msg
        PhantomIntelligencePixel.msg
        PhantomIntelligenceTrack.msg
)

add_service_files(
        DIRECTORY srv
        FILES SendCommandToSensor.srv
)

generate_messages(DEPENDENCIES std_msgs)

#######################################################################################

######### catkin specific configuration ###############################################
catkin_package(
        INCLUDE_DIRS include ${EXTERNAL_GATEWAY_INCLUDE_DIR}
        LIBRARIES ${GATEWAY_DRIVER_LIB}
        CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
        DEPENDS ${EXTERNAL_GATEWAY_LIB_NAME}
)
#######################################################################################

include_directories(
        include
        ${EXTERNAL_GATEWAY_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_LIBRARY_DIRS})

##### Driver Gateway Lib ##############################################################
file(GLOB GATEWAY_DRIVER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/gateway_driver/*")

add_library(${GATEWAY_DRIVER_LIB} ${GATEWAY_DRIVER_FILES} src/gazebo_driver/awl16_gazebo_ros_translator.cpp)

add_dependencies(${GATEWAY_DRIVER_LIB}
        ${PROJECT_NAME}_generate_messages_cpp
        ${${GATEWAY_DRIVER_LIB}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(${GATEWAY_DRIVER_LIB}
        ${EXTERNAL_GATEWAY_LIB_NAME}
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        )

add_executable(gazebo_awl16_node src/gazebo_driver/awl16_gazebo_ros_translator.cpp)
link_directories(${GAZEBO_LIBRARY_DIRS})
target_link_libraries(gazebo_awl16_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(CATKIN_DEPENDS gazebo_msgs message_runtime)
#######################################################################################

##### AWL16 Node ######################################################################
file(GLOB AWL16_NODE_DRIVER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/awl16_node.cpp")
add_executable(${AWL16_NODE}
        ${AWL16_NODE_DRIVER_FILES}
        ${catkin_LIBRARIES}
        )

add_dependencies(${AWL16_NODE}
        ${PROJECT_NAME}_generate_messages_cpp
        ${${AWL16_NODE}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(${AWL16_NODE}
        ${EXTERNAL_GATEWAY_LIB_NAME}
        ${GATEWAY_DRIVER_LIB}
        ${catkin_LIBRARIES}
        )
#######################################################################################

##### Guardian Node ######################################################################
file(GLOB GUARDIAN_NODE_DRIVER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/guardian_node.cpp")
add_executable(${GUARDIAN_NODE}
        ${GUARDIAN_NODE_DRIVER_FILES}
        ${catkin_LIBRARIES}
        )

add_dependencies(${GUARDIAN_NODE}
        ${PROJECT_NAME}_generate_messages_cpp
        ${${GUARDIAN_NODE}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(${GUARDIAN_NODE}
        ${EXTERNAL_GATEWAY_LIB_NAME}
        ${GATEWAY_DRIVER_LIB}
        ${catkin_LIBRARIES}
        )
#######################################################################################

##### Gazebo Plugins ######################################################################

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        gazebo_plugins
        gazebo_ros
        roscpp
        )
find_package(gazebo REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp
)
## Specify additional locations of header files

include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${GAZEBO_INCLUDE_DIRS}
        )

add_library(awl16_gazebo_plugin src/gazebo_driver/phantom_gazebo_plugins/gazebo_ros_awl16.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(awl16_gazebo_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} RayPlugin)





############################################################################################
## Mark executables and libraries for installation
install(TARGETS
        ${AWL16_NODE}
        ${GUARDIAN_NODE}
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
    find_package(catkin REQUIRED COMPONENTS rostest roslaunch)

    file(GLOB TEST_FILES ${CMAKE_CURRENT_SOURCE_DIR}/test/*)
    catkin_add_gtest(${TEST_EXECUTABLE} ${TEST_FILES})
    add_dependencies(${TEST_EXECUTABLE}
            ${PROJECT_NAME}_generate_messages_cpp
            )
    target_link_libraries(${TEST_EXECUTABLE}
            ${EXTERNAL_GATEWAY_LIB_NAME}
            ${GATEWAY_DRIVER_LIB}
            ${catkin_LIBRARIES})

    roslaunch_add_file_check(launch/run_tests.launch)
endif ()

